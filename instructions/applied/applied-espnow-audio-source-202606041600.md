# 事後承認 note: ESP-NOW 音声ソース + Studio JSON 設定 実装・build-verify 済 (DEC-034 Phase 5b)

- **編集元セッション**: hapbeat-sdk-workspace (起点 repo)、2026-06-04〜
- **関連**: 同 repo `instructions/instructions-espnow-audio-source-202606041600.md` の Phase 1〜2 を workspace セッションが直接実装 + PlatformIO build 検証した分の事後承認。
- **commit**: `58c63a0`

## 入った変更

1. **`src/node_serial_config.{h,cpp}`** — Studio Web Serial 用 JSON 行プロトコル（contracts `serial-config.md`）。先頭 `{` を peek して binary Bridge frame と共存。対応: `get_info`（role=transmitter / transport=espnow_stream / board=m5stack_core / espnow_channel / input_level）、`set_espnow_channel`（1/6/11 検証 + live apply + NVS `espnow/channel`）、`set_input_level`（0..100 + NVS `tx/input_level`）、`reboot`。
2. **`src/audio_source.{h,cpp}`**（`#ifdef AUDIO_SOURCE`）— ES8388 line-in → I2S 48kHz stereo 取込 → box-filter decimate 16kHz → input_level gain → IMA-ADPCM encode → **0xAA stream packet**（header に per-packet predictor/step、`espnow-stream.md` §3.1 準拠）を `esp_now_send(BROADCAST_MAC, ...)`。
3. **`src/ima_adpcm.{h,cpp}`** — encoder。**STEP/INDEX table と algorithm は device-firmware の ima_adpcm.cpp と同一**（受信側 `espnow_stream.cpp` のデコード整合を保証）。
4. **`src/espnow_sender.cpp`** — channel を **NVS 優先**（`espnow/channel`、無効値は compile-time 既定にフォールバック）。peer channel も同値。Studio からの設定が再起動を跨ぐ。
5. **`src/main.cpp`** — `nodeSerialConfigUpdate()` を loop 先頭で共通実行（relay build でも JSON config 可能）。`AUDIO_SOURCE` 時は Bridge protocol を使わず `Serial.begin(921600)` + `audioSourceSetup/Loop` に dispatch。
6. **`platformio.ini`** — `m5stack_audio_tx` env 新設（`-DAUDIO_SOURCE`）+ 両 env に ArduinoJson。

## 検証状況

- **ビルド**: `pio run -e esp32`（relay 回帰）+ `-e m5stack_audio_tx` **両 SUCCESS**（main.cpp 強制再コンパイルで baud 修正込みを確認）。
- **実機**: 未。**特に ES8388 のレジスタ初期化列と I2S pin map（MCLK=0/BCLK=13/LRCK=12/DIN=34）は M5 Module-Audio 参考値**であり、実基板での bring-up（ライン入力レベル・入力セレクト）が必要。

## この repo セッションへのアクション

1. 実機確認: Studio onboarding「ライブ送信機」→ `m5stack_audio_tx` 書込 → USB 再接続 → ESP-NOW subtab で channel/入力レベル設定 → get_info 反映。
2. PA ライン入力 → device-firmware `necklace_v3_stream_espnow` 受信機（複数台）で同時振動（channel 一致）。ES8388 レジスタを実機で詰める。
3. piggyback（単発ロス回復、`espnow-stream.md` §3.2）は未実装（受信側は piggyback 無しを許容）。損失が問題になれば送信側に追加。
4. 問題なければ本 note + 元 instruction を `instructions/completed/` へ。
5. 配布物 repo のため push / tag はユーザー実機検証後。
