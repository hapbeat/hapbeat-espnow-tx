#include "display.h"
#include <M5Stack.h>

// Layout constants
static constexpr int HEADER_Y      = 0;
static constexpr int HEADER_H      = 30;
static constexpr int STATUS_Y      = 35;
static constexpr int REACTION_Y    = 130;
static constexpr int REACTION_H    = 110;
static constexpr int SCREEN_W      = 320;

// Reaction display timeout (ms)
static constexpr uint32_t REACTION_TIMEOUT_MS = 1500;

// State
static uint32_t s_reaction_shown_at = 0;
static bool     s_reaction_visible  = false;

// Previous status values for dirty-check (avoid full redraw)
static uint32_t s_prev_uptime  = UINT32_MAX;
static uint32_t s_prev_sent    = UINT32_MAX;
static uint32_t s_prev_failed  = UINT32_MAX;
static uint8_t  s_prev_wifi_ch = 0xFF;
static int8_t   s_prev_synced  = -1;  // -1 = never drawn
static bool     s_status_first = true; // first draw needs full clear
static int8_t   s_prev_connected = -1; // -1 = never drawn

// Color helpers (RGB565)
static constexpr uint16_t COL_BG       = TFT_BLACK;
static constexpr uint16_t COL_HEADER   = TFT_NAVY;
static constexpr uint16_t COL_DARKGREEN = 0x03E0; // dark green in RGB565
static constexpr uint16_t COL_TEXT     = TFT_WHITE;
static constexpr uint16_t COL_OK       = TFT_GREEN;
static constexpr uint16_t COL_FAIL     = TFT_RED;
static constexpr uint16_t COL_WARN     = TFT_YELLOW;
static constexpr uint16_t COL_DIM      = TFT_DARKGREY;

void displayInit() {
    M5.Lcd.fillScreen(COL_BG);
    M5.Lcd.setTextColor(COL_TEXT, COL_BG);

    // Draw header bar
    M5.Lcd.fillRect(0, HEADER_Y, SCREEN_W, HEADER_H, COL_HEADER);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(COL_TEXT, COL_HEADER);
    M5.Lcd.drawString("HAPBEAT TX", SCREEN_W / 2, HEADER_Y + HEADER_H / 2);

    M5.Lcd.setTextDatum(TL_DATUM);
    M5.Lcd.setTextColor(COL_TEXT, COL_BG);
}

void displayUpdateHeader(bool connected) {
    int8_t val = connected ? 1 : 0;
    if (val == s_prev_connected) return;
    s_prev_connected = val;

    uint16_t bar_col = connected ? COL_DARKGREEN : COL_HEADER;
    M5.Lcd.fillRect(0, HEADER_Y, SCREEN_W, HEADER_H, bar_col);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(COL_TEXT, bar_col);

    const char* label = connected ? "HAPBEAT TX  Connected" : "HAPBEAT TX  Disconnected";
    M5.Lcd.drawString(label, SCREEN_W / 2, HEADER_Y + HEADER_H / 2);

    M5.Lcd.setTextDatum(TL_DATUM);
    M5.Lcd.setTextColor(COL_TEXT, COL_BG);
}

void displayBootStatus(const char* msg) {
    // Show boot messages in the status area
    static int boot_line = 0;
    int y = STATUS_Y + boot_line * 20;
    if (y > 220) {
        // Scroll: clear status area and reset
        M5.Lcd.fillRect(0, STATUS_Y, SCREEN_W, REACTION_Y - STATUS_Y, COL_BG);
        boot_line = 0;
        y = STATUS_Y;
    }
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(COL_DIM, COL_BG);
    M5.Lcd.setCursor(4, y);
    M5.Lcd.print("> ");
    M5.Lcd.println(msg);
    boot_line++;
}

// Helper: overwrite a fixed-width field at (x,y) with background color to avoid residue.
// Uses setTextSize(2) = 12px wide per char.
static void drawField(int x, int y, uint16_t color, const char* text, int field_chars) {
    // Pad with spaces to fixed width so old text is overwritten
    char buf[32];
    int len = strlen(text);
    if (len > field_chars) len = field_chars;
    memcpy(buf, text, len);
    for (int i = len; i < field_chars && i < 31; i++) buf[i] = ' ';
    buf[(field_chars < 31) ? field_chars : 31] = '\0';

    M5.Lcd.setTextColor(color, COL_BG);
    M5.Lcd.setCursor(x, y);
    M5.Lcd.print(buf);
}

void displayUpdateStatus(uint32_t uptime_sec, uint32_t sent, uint32_t failed,
                         uint8_t wifi_ch, bool time_synced) {
    // Clear reaction area if timeout elapsed
    if (s_reaction_visible && (millis() - s_reaction_shown_at > REACTION_TIMEOUT_MS)) {
        M5.Lcd.fillRect(0, REACTION_Y, SCREEN_W, REACTION_H, COL_BG);
        s_reaction_visible = false;
    }

    M5.Lcd.setTextSize(2);

    // On first draw, clear the status area once
    if (s_status_first) {
        M5.Lcd.fillRect(0, STATUS_Y, SCREEN_W, REACTION_Y - STATUS_Y - 2, COL_BG);
        s_status_first = false;
    }

    // Line 1: Uptime — only redraw if changed
    int y = STATUS_Y;
    if (uptime_sec != s_prev_uptime) {
        s_prev_uptime = uptime_sec;
        uint32_t h = uptime_sec / 3600;
        uint32_t m = (uptime_sec % 3600) / 60;
        uint32_t s = uptime_sec % 60;
        char tmp[20];
        snprintf(tmp, sizeof(tmp), "Up: %02u:%02u:%02u", h, m, s);
        drawField(4, y, COL_TEXT, tmp, 16);
    }

    // Line 2: Sent / Failed — only redraw if changed
    y += 24;
    if (sent != s_prev_sent || failed != s_prev_failed) {
        s_prev_sent = sent;
        s_prev_failed = failed;
        char tmp[30];
        if (failed > 0) {
            snprintf(tmp, sizeof(tmp), "TX:%-6u ERR:%-4u", sent, failed);
        } else {
            snprintf(tmp, sizeof(tmp), "TX:%-6u", sent);
        }
        // Green for TX count, but use a single drawField for simplicity
        // (color the whole line based on whether errors exist)
        drawField(4, y, (failed > 0) ? COL_WARN : COL_OK, tmp, 22);
    }

    // Line 3: WiFi channel + Time sync — only redraw if changed
    y += 24;
    int8_t synced_val = time_synced ? 1 : 0;
    if (wifi_ch != s_prev_wifi_ch || synced_val != s_prev_synced) {
        s_prev_wifi_ch = wifi_ch;
        s_prev_synced = synced_val;

        char tmp[8];
        snprintf(tmp, sizeof(tmp), "CH: %u", wifi_ch);
        drawField(4, y, COL_TEXT, tmp, 6);

        if (time_synced) {
            drawField(160, y, COL_OK, "SYNC: OK", 10);
        } else {
            drawField(160, y, COL_WARN, "SYNC: --", 10);
        }
    }
}

void displaySendReaction(const char* cmd, const char* event_id,
                         uint8_t group, bool success) {
    // Flash the reaction area
    uint16_t bg_col = success ? COL_DARKGREEN : TFT_MAROON;
    M5.Lcd.fillRect(0, REACTION_Y, SCREEN_W, REACTION_H, bg_col);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextDatum(MC_DATUM);

    // Command name with result indicator
    M5.Lcd.setTextColor(success ? COL_OK : COL_FAIL, bg_col);
    M5.Lcd.drawString(cmd, SCREEN_W / 2, REACTION_Y + 20);

    // Event ID
    if (event_id && event_id[0] != '\0') {
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(COL_TEXT, bg_col);
        // At size 2, ~26 chars fit in 320px
        char buf[27];
        strncpy(buf, event_id, 26);
        buf[26] = '\0';
        M5.Lcd.drawString(buf, SCREEN_W / 2, REACTION_Y + 55);
    }

    // Group info
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(COL_DIM, bg_col);
    char grp_buf[20];
    snprintf(grp_buf, sizeof(grp_buf), "group: %u", group);
    M5.Lcd.drawString(grp_buf, SCREEN_W / 2, REACTION_Y + 85);

    M5.Lcd.setTextDatum(TL_DATUM);
    M5.Lcd.setTextColor(COL_TEXT, COL_BG);

    s_reaction_shown_at = millis();
    s_reaction_visible = true;
}

void displayError(const char* msg) {
    M5.Lcd.fillRect(0, REACTION_Y, SCREEN_W, REACTION_H, TFT_MAROON);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.setTextColor(COL_FAIL, TFT_MAROON);
    M5.Lcd.drawString("ERROR", SCREEN_W / 2, REACTION_Y + 20);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(COL_TEXT, TFT_MAROON);
    M5.Lcd.drawString(msg, SCREEN_W / 2, REACTION_Y + 50);
    M5.Lcd.setTextDatum(TL_DATUM);
    M5.Lcd.setTextColor(COL_TEXT, COL_BG);

    s_reaction_shown_at = millis();
    s_reaction_visible = true;
}
