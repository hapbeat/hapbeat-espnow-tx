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
        r["espnow_channel"] = p.getUChar("channel", 11);   // NVS-empty default (= config.h ESPNOW_CHANNEL)

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
        {
            // input_mix (DEC-046 follow-up): CoreS3 line-in mono-ize workaround —
            // see audio_source.cpp s_input_mix docstring. Read straight from NVS
            // (like input_level above) so get_info is correct even before the
            // AUDIO_SOURCE-only live setter has run this boot.
            static const char* mixNames[3] = {"stereo", "mono_l", "mono_r"};
            uint8_t mixv = p.getUChar("input_mix", 0);
            r["input_mix"] = mixNames[mixv > 2 ? 0 : mixv];
        }
        // opus_complexity (DEC-046 follow-up, set_opus_complexity): GLOBAL
        // Opus encoder complexity override (mode 9 SOLID48 tuning) — see
        // audio_source.cpp s_opus_complexity_override docstring. -1 = unset
        // (each mode's MODE_DEFS complexity applies). Read straight from NVS
        // (like input_level/input_mix above) so get_info is correct even
        // before the AUDIO_SOURCE-only live setter has run this boot.
        r["opus_complexity"] = p.getInt("opus_cplx", -1);
        p.end();
#ifdef AUDIO_SOURCE
        // LONGRANGE state (DEC-043 P5). Only the live audio source carries it.
        r["range"]      = audioSourceGetRange() ? "long" : "normal";
        r["lr_preset"]  = audioSourceGetLrPreset();
        r["lr_bitrate"] = audioSourceGetLrBitrate();
        // input_sel (EXPERIMENTAL, set_input_sel — see audio_source.cpp
        // s_input_sel docstring). NOT NVS-backed like input_mix above: this
        // reads the LIVE in-RAM value (always "in1" on classic Core, where
        // audioSourceApplyInputSel is a no-op).
        r["input_sel"]  = audioSourceGetInputSelName();
        // stream_hp_buffer_ms (DEC-046 follow-up, set_stream_hp_buffer):
        // mode-9 SOLID48 receiver HP jitter-buffer depth, broadcast as 0xAC
        // fleet param 6 — see audioSourceSetStreamHpBuffer's docstring in
        // audio_source.cpp. AUDIO_SOURCE-only (uses the fleet-tune mechanism,
        // which is private to audio_source.cpp), unlike opus_complexity above.
        r["stream_hp_buffer_ms"] = audioSourceGetStreamHpBuffer();
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

    // ---- set_input_mix -------------------------------------------------
    // {"cmd":"set_input_mix","mix":"stereo"|"mono_l"|"mono_r"} — CoreS3
    // line-in mono-ize workaround for a HW-defective L channel (feeds the
    // healthy channel to both sides). See audio_source.cpp s_input_mix
    // docstring. Persisted; applied live, no reboot (unlike set_stream_mode).
    if (strcmp(cmd, "set_input_mix") == 0) {
        const char* mix = doc["mix"] | "";
        int mixVal;
        if (strcmp(mix, "stereo") == 0) mixVal = 0;
        else if (strcmp(mix, "mono_l") == 0) mixVal = 1;
        else if (strcmp(mix, "mono_r") == 0) mixVal = 2;
        else {
            r["status"] = "error"; r["cmd"] = cmd;
            r["message"] = "mix must be 'stereo'/'mono_l'/'mono_r'";
            sendResp(r); return;
        }
        Preferences p; p.begin("tx", false); p.putUChar("input_mix", (uint8_t)mixVal); p.end();
#ifdef AUDIO_SOURCE
        audioSourceApplyInputMix(mixVal);   // live: no reboot, see docstring
#endif
        r["status"] = "ok"; r["cmd"] = cmd; r["mix"] = mix;
        sendResp(r);
        return;
    }

    // ---- set_opus_complexity -----------------------------------------------
    // {"cmd":"set_opus_complexity","value":0..10} — GLOBAL override of the
    // Opus encoder complexity (OPUS_SET_COMPLEXITY) for whichever Opus mode
    // is currently streaming (mode 9 SOLID48 in particular — DEC-046
    // follow-up). Unset (never called this boot) falls back to each mode's
    // MODE_DEFS complexity. Persisted; applied live to the running encoder,
    // no reboot needed (mirrors set_input_level / set_input_mix's NVS +
    // live-apply + get_info shape above).
    if (strcmp(cmd, "set_opus_complexity") == 0) {
        int cplx = doc["value"] | -1;
        // -1 = "auto" (fall back to each mode's MODE_DEFS complexity); 0..10 = override.
        if (cplx < -1 || cplx > 10) {
            r["status"] = "error"; r["cmd"] = cmd;
            r["message"] = "value must be -1 (auto) or 0..10";
            sendResp(r); return;
        }
        Preferences p; p.begin("tx", false); p.putInt("opus_cplx", cplx); p.end();
#ifdef AUDIO_SOURCE
        audioSourceApplyOpusComplexity(cplx);   // live: re-ctl's the running encoder
#endif
        r["status"] = "ok"; r["cmd"] = cmd; r["opus_complexity"] = cplx;
        sendResp(r);
        return;
    }

    // ---- set_input_sel — EXPERIMENTAL hardware-debug command ---------------
    // {"cmd":"set_input_sel","sel":"in1"|"in2"|"diff1"} — hot-switches the
    // ES8388 ADC input mux (CoreS3 only) to investigate a frequency-shelf
    // attenuation defect on the line-in L channel. See audio_source.cpp
    // s_input_sel docstring for the full hypothesis + the 3 values this
    // enumerates (es_adc_input_t has exactly these 3 — no others exist).
    // NOT persisted to NVS: applied live only, a reboot restores in1.
    if (strcmp(cmd, "set_input_sel") == 0) {
        const char* sel = doc["sel"] | "";
        int selVal;
        if (strcmp(sel, "in1") == 0) selVal = 0;        // ADC_INPUT_LINPUT1_RINPUT1
        else if (strcmp(sel, "in2") == 0) selVal = 1;   // ADC_INPUT_LINPUT2_RINPUT2
        else if (strcmp(sel, "diff1") == 0) selVal = 2; // ADC_INPUT_DIFFERENCE1
        else {
            r["status"] = "error"; r["cmd"] = cmd;
            r["message"] = "sel must be 'in1'/'in2'/'diff1'";
            sendResp(r); return;
        }
#ifdef AUDIO_SOURCE
        if (audioSourceApplyInputSel(selVal)) {
            r["status"] = "ok"; r["cmd"] = cmd; r["sel"] = sel;
        } else {
            r["status"] = "error"; r["cmd"] = cmd;
            r["message"] = "not supported (classic Core has no addressable ADC input mux, or codec not ready)";
        }
#else
        r["status"] = "error"; r["cmd"] = cmd; r["message"] = "not a source";
#endif
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
    // set_lr_preset {"preset":0..5} → enter LONGRANGE with an LR preset + reboot
    // (canonical; ECO/ECO-SAFE/STD/STD-SAFE/HQ/HQ-SAFE).
    if (strcmp(cmd, "set_lr_preset") == 0) {
        int pr = doc["preset"] | -1;
        if (pr < 0 || pr > 5) {
            r["status"] = "error"; r["cmd"] = cmd; r["message"] = "preset must be 0..5";
            sendResp(r); return;
        }
        r["status"] = "ok"; r["cmd"] = cmd; r["preset"] = pr;
        sendResp(r);
        audioSourceSetLrPreset(pr);
        return;
    }
    // set_lr_bitrate {"bitrate":24000|32000|48000} → deprecated alias → set_lr_preset.
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
    // set_fleet_param {"param":1-6,"value":0-255} → broadcast a 0xAC beacon
    // (5 = receiver volume_max %, 6 = mode-9 SOLID48 HP buffer deci-ms, §3.5).
    // 1=buffer_ms 2=selection 3=lock_timeout(×10ms) 4=resync_gap (§3.5).
    if (strcmp(cmd, "set_fleet_param") == 0) {
        int param = doc["param"] | 0;
        int value = doc["value"] | 0;
        // param 6 (mode-9 HP buffer) has a dedicated command (set_stream_hp_buffer)
        // that keeps the NVS + get_info stream_hp_buffer_ms in sync; the raw path
        // would broadcast a deci-ms value without updating them, so reject it here.
        if (param == 6) {
            r["status"] = "error"; r["cmd"] = cmd;
            r["message"] = "use set_stream_hp_buffer for param 6";
            sendResp(r); return;
        }
        audioSourceSetFleetParam(param, value);
        r["status"] = "ok"; r["cmd"] = cmd; r["param"] = param; r["value"] = value;
        sendResp(r);
        return;
    }
    // set_stream_hp_buffer {"value":40..500} → mode-9 SOLID48 receiver HP
    // jitter-buffer depth in ms (DEC-046 follow-up). Persists the raw ms +
    // broadcasts it to the fleet as 0xAC param 6 (wire = clamp(ms/10,4,50)
    // deci-ms), reusing the exact same burst ×3 + 5 s resend + epoch
    // mechanism params 1-5 use (audioSourceSetFleetParam).
    if (strcmp(cmd, "set_stream_hp_buffer") == 0) {
        int ms = doc["value"] | 120;
        if (ms < 40 || ms > 500) {
            r["status"] = "error"; r["cmd"] = cmd;
            r["message"] = "value must be 40..500";
            sendResp(r); return;
        }
        audioSourceSetStreamHpBuffer(ms);
        r["status"] = "ok"; r["cmd"] = cmd;
        r["stream_hp_buffer_ms"] = audioSourceGetStreamHpBuffer();
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
