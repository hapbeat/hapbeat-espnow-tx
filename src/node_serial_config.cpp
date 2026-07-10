// ---------------------------------------------------------------------------
// node_serial_config.cpp — Studio JSON config for the transmitter (see header).
// ---------------------------------------------------------------------------

#include "node_serial_config.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_wifi.h>

#ifdef AUDIO_SOURCE
#include "audio_source.h"
#endif
#ifdef REPEATER
#include "repeater.h"
#endif

// FIRMWARE_VERSION / BUILD_COMMIT_SHA are auto-generated per env by
// scripts/build_version.py (pre-build) from firmware-versions.json (DEC-035).
// The header is gitignored and regenerated on every build.
#include "build_version.h"

// Determine build mode string at compile time.
#if defined(AUDIO_SOURCE)
  static const char* BUILD_MODE = "source";
#elif defined(REPEATER)
  static const char* BUILD_MODE = "repeater";
#else
  static const char* BUILD_MODE = "relay";
#endif

static void sendResp(JsonDocument& r) {
    String out;
    serializeJson(r, out);
    Serial.print(out);
    Serial.print('\n');
}

// Parse "AA:BB:CC:DD:EE:FF" (case-insensitive) → 6 bytes.
// Returns true on success.
static bool parseMacStr(const char* str, uint8_t* mac) {
    unsigned int b[6];
    if (sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
               &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) return false;
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)b[i];
    return true;
}

static void macToStr(const uint8_t* mac, char* buf, size_t n) {
    snprintf(buf, n, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// board string reported in get_info — must match gen_variant.py's VARIANTS
// table (scripts/gen_variant.py) so Studio's firmware-library board check
// and device-list board readout agree on the same identifiers.
#if defined(BOARD_XIAO_C6)
static const char* BOARD_ID = "xiao_c6";
#elif defined(BOARD_CORES3)
static const char* BOARD_ID = "m5stack_cores3";
#else
static const char* BOARD_ID = "m5stack_basic";
#endif

// Build the default name "<MAC4>-<board>" (underscores -> hyphens), mirroring
// hapbeat-device-firmware/src/device_identity.h so a transmitter card reads
// the same way as a receiver card in the Studio device list.
static String deviceDefaultName() {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char buf[40];
    snprintf(buf, sizeof(buf), "%02X%02X-%s", mac[4], mac[5], BOARD_ID);
    for (char* c = buf; *c; c++) {
        if (*c == '_') *c = '-';
    }
    return String(buf);
}

// The user-set name from NVS ("tx" namespace, key "dev_name"), or the
// board+MAC default when unset.
static String deviceResolvedName() {
    Preferences p;
    p.begin("tx", true);
    String name = p.getString("dev_name", "");
    p.end();
    return name.length() > 0 ? name : deviceDefaultName();
}

static void handleLine(const char* line) {
    JsonDocument doc;
    if (deserializeJson(doc, line)) return;   // not valid JSON — ignore
    const char* cmd = doc["cmd"] | "";

    JsonDocument r;

    // ---- get_info ------------------------------------------------------
    // Flat top-level shape (device-firmware serial_config.cpp cmdGetInfo
    // parity) — Studio's parseSerialInfo (serialMaster.ts) reads name/fw/
    // role/transport/board straight off the response, not nested under a
    // "data" object. "fw" (not "firmware") is the key name Studio expects.
    if (strcmp(cmd, "get_info") == 0) {
        r["status"]    = "ok";
        r["cmd"]       = "get_info";
        r["name"]      = deviceResolvedName();
        r["mac"]       = WiFi.macAddress();
        r["fw"]        = FIRMWARE_VERSION;
        r["role"]      = "transmitter";
        r["transport"] = "espnow_stream";
        r["board"]     = BOARD_ID;
        r["mode"]      = BUILD_MODE;   // "source" | "repeater" | "relay"

        Preferences p;
        p.begin("espnow", true);
        r["espnow_channel"] = p.getUChar("channel", 1);

        // relay_src: MAC string when pinned, null when auto origin-follow
        // (DEC-043). An all-zero stored MAC = auto, reported as null.
        uint8_t relay_mac[6] = {};
        bool pinned = false;
        if (p.getBytesLength("relay_src") == 6) {
            p.getBytes("relay_src", relay_mac, 6);
            for (int i = 0; i < 6; i++) if (relay_mac[i]) { pinned = true; break; }
        }
        if (pinned) {
            char mac_str[18];
            macToStr(relay_mac, mac_str, sizeof(mac_str));
            r["relay_src"] = mac_str;
        } else {
            r["relay_src"] = nullptr;
        }
        p.end();

        p.begin("tx", true);
        r["input_level"] = p.getInt("input_level", 50);
        p.end();
#ifdef AUDIO_SOURCE
        // LONGRANGE state (DEC-043 P5). Only the live audio source carries it.
        r["range"]      = audioSourceGetRange() ? "long" : "normal";
        r["lr_bitrate"] = audioSourceGetLrBitrate();
#endif

        sendResp(r);
        return;
    }

    // ---- set_name ------------------------------------------------------
    // Mirrors device-firmware serial_config.cpp cmdSetName: 32-char
    // truncate, persist to NVS ("tx"/"dev_name"), echo the stored value.
    if (strcmp(cmd, "set_name") == 0) {
        const char* name = doc["name"];
        if (!name) {
            r["status"] = "error"; r["cmd"] = cmd;
            r["message"] = "missing name";
            sendResp(r); return;
        }
        char truncated[33];
        strncpy(truncated, name, 32);
        truncated[32] = '\0';

        Preferences p; p.begin("tx", false);
        p.putString("dev_name", truncated);
        p.end();

        r["status"] = "ok"; r["cmd"] = cmd; r["name"] = truncated;
        sendResp(r);
        return;
    }

    // ---- set_espnow_channel ------------------------------------------------
    if (strcmp(cmd, "set_espnow_channel") == 0) {
        int ch = doc["channel"] | 1;
        if (ch != 1 && ch != 6 && ch != 11) {
            r["status"] = "error"; r["cmd"] = cmd;
            r["message"] = "channel must be 1/6/11";
            sendResp(r); return;
        }
        Preferences p; p.begin("espnow", false);
        p.putUChar("channel", (uint8_t)ch); p.end();
        esp_wifi_set_channel((uint8_t)ch, WIFI_SECOND_CHAN_NONE);  // live apply
        r["status"] = "ok"; r["cmd"] = cmd; r["channel"] = ch;
        sendResp(r);
        return;
    }

    // ---- set_input_level ---------------------------------------------------
    if (strcmp(cmd, "set_input_level") == 0) {
        int lv = doc["level"] | 50;
        if (lv < 0) lv = 0; if (lv > 100) lv = 100;
        Preferences p; p.begin("tx", false); p.putInt("input_level", lv); p.end();
#ifdef AUDIO_SOURCE
        audioSourceApplyInputLevel(lv);   // live: drives ES8388 analog PGA on CoreS3
#endif
        r["status"] = "ok"; r["cmd"] = cmd; r["level"] = lv;
        sendResp(r);
        return;
    }

    // ---- set_stream_mode (0=SOLID..8=FINE; see espnow-stream.md §3.4) --
    if (strcmp(cmd, "set_stream_mode") == 0) {
        int m = doc["mode"] | 0;
#ifdef AUDIO_SOURCE
        audioSourceSetMode(m);            // persists NVS + live-switches the encoder
        r["status"] = "ok"; r["cmd"] = cmd; r["mode"] = audioSourceGetMode();
#else
        r["status"] = "error"; r["cmd"] = cmd; r["message"] = "not a source";
#endif
        sendResp(r);
        return;
    }

#ifdef AUDIO_SOURCE
    // ---- LONGRANGE + fleet-tune (DEC-043 P5) -------------------------------
    // set_range_mode {"long":bool} → NVS tx/range_long + reboot (CoreS3 only).
    if (strcmp(cmd, "set_range_mode") == 0) {
        bool lng = doc["long"] | false;
        r["status"] = "ok"; r["cmd"] = cmd; r["range"] = lng ? "long" : "normal";
        sendResp(r);                       // reply BEFORE the reboot inside the setter
        audioSourceSetRange(lng);
        return;
    }
    // set_lr_bitrate {"bitrate":24000|32000|48000} → NVS + reboot (LR profile).
    if (strcmp(cmd, "set_lr_bitrate") == 0) {
        int br = doc["bitrate"] | 24000;
        if (br != 24000 && br != 32000 && br != 48000) {
            r["status"] = "error"; r["cmd"] = cmd;
            r["message"] = "bitrate must be 24000/32000/48000";
            sendResp(r); return;
        }
        r["status"] = "ok"; r["cmd"] = cmd; r["bitrate"] = br;
        sendResp(r);
        audioSourceSetLrBitrate(br);
        return;
    }
    // set_fleet_param {"param":1-4,"value":0-255} → broadcast a 0xAC beacon.
    // 1=buffer_ms 2=selection 3=lock_timeout(×10ms) 4=resync_gap (§3.5).
    if (strcmp(cmd, "set_fleet_param") == 0) {
        int param = doc["param"] | 0;
        int value = doc["value"] | 0;
        audioSourceSetFleetParam(param, value);
        r["status"] = "ok"; r["cmd"] = cmd; r["param"] = param; r["value"] = value;
        sendResp(r);
        return;
    }
#endif

    // ---- set_relay_source --------------------------------------------------
    // Sets the source MAC to relay (for REPEATER builds; stored for all modes
    // so it survives a reflash to REPEATER firmware without reconfiguration).
    // {"cmd":"set_relay_source","mac":"AA:BB:CC:DD:EE:FF"}
    if (strcmp(cmd, "set_relay_source") == 0) {
        const char* mac_str = doc["mac"] | "";
        uint8_t mac[6] = {};
        if (!parseMacStr(mac_str, mac)) {
            r["status"] = "error"; r["cmd"] = cmd;
            r["message"] = "mac must be 'AA:BB:CC:DD:EE:FF'";
            sendResp(r); return;
        }
        // Persist to NVS
        Preferences p; p.begin("espnow", false);
        p.putBytes("relay_src", mac, 6); p.end();
        // Live apply in REPEATER build
#ifdef REPEATER
        repeaterSetRelaySrc(mac);
#endif
        char out_str[18];
        macToStr(mac, out_str, sizeof(out_str));
        r["status"]    = "ok";
        r["cmd"]       = cmd;
        r["relay_src"] = out_str;
        sendResp(r);
        return;
    }

    // ---- reboot ------------------------------------------------------------
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
