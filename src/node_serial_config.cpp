// ---------------------------------------------------------------------------
// node_serial_config.cpp — Studio JSON config for the transmitter (see header).
// ---------------------------------------------------------------------------

#include "node_serial_config.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_wifi.h>

static const char* FW_VERSION = "0.1.0";

static void sendResp(JsonDocument& r) {
    String out;
    serializeJson(r, out);
    Serial.print(out);
    Serial.print('\n');
}

static void handleLine(const char* line) {
    JsonDocument doc;
    if (deserializeJson(doc, line)) return;   // not valid JSON — ignore
    const char* cmd = doc["cmd"] | "";

    JsonDocument r;

    if (strcmp(cmd, "get_info") == 0) {
        r["status"] = "ok";
        r["cmd"]    = "get_info";
        JsonObject d = r["data"].to<JsonObject>();
        d["name"]      = "hapbeat-transmitter";
        d["mac"]       = WiFi.macAddress();
        d["firmware"]  = FW_VERSION;
        d["role"]      = "transmitter";
        d["transport"] = "espnow_stream";
        d["board"]     = "m5stack_core";
        Preferences p;
        p.begin("espnow", true);
        d["espnow_channel"] = p.getUChar("channel", 1);
        p.end();
        p.begin("tx", true);
        d["input_level"] = p.getInt("input_level", 50);
        p.end();
        sendResp(r);
        return;
    }

    if (strcmp(cmd, "set_espnow_channel") == 0) {
        int ch = doc["channel"] | 1;
        if (ch != 1 && ch != 6 && ch != 11) {
            r["status"] = "error"; r["cmd"] = cmd; r["message"] = "channel must be 1/6/11";
            sendResp(r); return;
        }
        Preferences p; p.begin("espnow", false); p.putUChar("channel", (uint8_t)ch); p.end();
        esp_wifi_set_channel((uint8_t)ch, WIFI_SECOND_CHAN_NONE);   // live apply
        r["status"] = "ok"; r["cmd"] = cmd; r["channel"] = ch;
        sendResp(r);
        return;
    }

    if (strcmp(cmd, "set_input_level") == 0) {
        int lv = doc["level"] | 50;
        if (lv < 0) lv = 0; if (lv > 100) lv = 100;
        Preferences p; p.begin("tx", false); p.putInt("input_level", lv); p.end();
        r["status"] = "ok"; r["cmd"] = cmd; r["level"] = lv;
        sendResp(r);
        return;
    }

    if (strcmp(cmd, "reboot") == 0) {
        r["status"] = "ok"; r["cmd"] = "reboot";
        sendResp(r);
        delay(200);
        ESP.restart();
        return;
    }

    r["status"] = "error"; r["cmd"] = "unknown"; r["message"] = "Unknown command";
    sendResp(r);
}

void nodeSerialConfigUpdate() {
    static char   buf[256];
    static size_t pos = 0;
    static bool   active = false;

    while (Serial.available()) {
        char c = (char)Serial.peek();
        if (!active && c != '{') return;   // binary frame — leave for SerialHandler
        active = true;
        Serial.read();                     // consume the peeked byte
        if (c == '\n' || pos >= sizeof(buf) - 1) {
            buf[pos] = 0;
            if (pos > 0) handleLine(buf);
            active = false;
            pos = 0;
            return;
        }
        if (c != '\r') buf[pos++] = c;
    }
}
