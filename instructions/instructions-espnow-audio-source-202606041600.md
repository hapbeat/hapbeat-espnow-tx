# 指示書: transmitter 役割 — ESP-NOW 音声ソースモード + シリアル設定 (Studio 連携)

- **起票元**: workspace セッション (hapbeat-sdk-workspace, 2026-06-04)
- **関連 DEC**: DEC-033 (ESP-NOW 60 台 streaming) / DEC-034 (ツールチェーン mode-aware)
- **依存仕様 (contracts、必読)**:
  - `hapbeat-contracts/specs/node-roles.md` — role/transport taxonomy + get_info 必須フィールド
  - `hapbeat-contracts/specs/serial-config.md` — 共通設定プロトコル（§4.9 set_espnow_channel / §4.11 set_input_level）
  - `hapbeat-contracts/specs/espnow-stream.md` — ESP-NOW stream packet 形式
  - `hapbeat-contracts/specs/firmware-distribution.md` — 配布 manifest v2

## 背景

DEC-034 で Studio / Helper / contracts を mode-aware 化済（build/typecheck 通過）。Studio は `role=transmitter` のノードに **ESP-NOW 設定 subtab**（channel + 入力レベル）と **ファーム subtab** を出す。
この repo は DEC-033 の「ライブ会場同報」の **ESP-NOW 音声送信機（transmitter 役割）** の home。
参考実装（**機能等価でよい・真似不要**）: `C:\GitHub\Hapbeat\wireless-sender-firmware`（ES8388/PCM1808 ライン入力 → 48k→16k decimate → IMA-ADPCM → ESP-NOW broadcast、in-flight throttle、piggyback）。

現状この repo は Bridge 駆動のコマンド中継（`espnow_sender.cpp` の固定 `EspNowPacket`）。**音声ソースモードは net-new**。

## 重要な制約

- **後方互換コードを作らない**（リリース前）。
- **配布物 repo**: 実機検証完了まで push / tag しない。実装は commit までで止める。
- PlatformIO は workspace に無いため未ビルド。実装者がこの repo セッションで `pio run` + 実機検証する。

---

## Phase 1 — シリアル JSON 設定プロトコル（Studio から設定可能にする・最優先）

Studio は transmitter を **USB Web Serial 直**で設定する（Wi-Fi STA 非接続のため Helper 経由不可）。`serial-config.md` の JSON 行プロトコル（先頭 `{` で JSON config にルーティング、921600 baud, `\n` 終端）を実装する。

`src/serial_handler.cpp` に JSON config パスを追加（既存のバイナリ Bridge プロトコルと先頭バイトで共存）。対応コマンド:

| cmd | 処理 | 永続化 |
|---|---|---|
| `get_info` | 下記 JSON を返す | — |
| `set_espnow_channel` | `channel` ∈ {1,6,11} を ESP-NOW ch に設定 | NVS `espnow/channel` |
| `set_input_level` | `level` ∈ [0,100] を codec の ADC/ライン入力ゲインに正規化反映 | NVS `tx/input_level` |
| `reboot` | 再起動 | — |
| `set_name` | 表示名（任意） | NVS |

`get_info` レスポンス（`node-roles.md` §4 準拠）:
```json
{"status":"ok","cmd":"get_info","data":{
  "name":"...", "mac":"...", "firmware":"...",
  "role":"transmitter", "transport":"espnow_stream",
  "board":"m5stack_core",        // or "xiao_c6" 等。Studio の board 検証用
  "espnow_channel":6, "input_level":50
}}
```

**確認**: Studio onboarding で「ライブ送信機」を選んで書込 → USB 再接続 → ESP-NOW subtab で channel / 入力レベルを設定 → get_info に反映。

---

## Phase 2 — ESP-NOW 音声ソースモード

仕様: `espnow-stream.md`。ライン入力 → ADPCM → ESP-NOW broadcast を `wireless-sender-firmware` から機能等価で移植。

- 入力: ライン入力 codec（M5 Module-Audio ES8388 / XIAO + PCM1808 等）。基板に応じた I2S RX。
- 48kHz 取込 → ソフト decimate 3× → 16kHz stereo。
- 16 frames/packet で IMA-ADPCM エンコード → ESP-NOW broadcast（peer `FF:FF:FF:FF:FF:FF`）。packet 形式・piggyback は `espnow-stream.md` §3（`lib/ima_adpcm` 相当）。
- 無線: 11b/g/n, 6Mbps, broadcast。in-flight throttle（stale より drop）。
- channel は Phase 1 の NVS 値。**受信機（device-firmware の espnow_stream 役割）と channel を一致**させる。
- DEC-033 のマルチソース（送信元多重化・独立クローン N 台）は将来拡張。本指示は単一ソース形式が基底。source-id を packet に足す拡張は DEC-033 実装フェーズで別途。

**確認**: PA ライン入力 → 送信機 → device-firmware 受信機（複数）が同時に音声ベース振動。

---

## Phase 3 — 配布 (firmware-distribution.md v2)

ビルド成果物に `variant.json` を出力:
```json
{ "role":"transmitter", "transport":"espnow_stream",
  "board":"m5stack_core", "label":"ESP-NOW 送信機（PA 入力）",
  "description":"PA ライン入力を ESP-NOW で同報する送信機。" }
```
CI release に bin + `manifest.fragment.json`（schema_version 2, 自 repo 分の variants）を添付。Studio dev plugin は workspace の `../hapbeat-transmitter-firmware/.pio/build` を読む設定済（`repo:'tx'`）。

transmitter は Wi-Fi 非接続なので `appOta` は不要・`fullSerial`（merged）のみでよい。

---

## 検証 (acceptance)

1. `pio run -e <audio source env>` ビルド成功。
2. Studio から「ライブ送信機」書込 → ESP-NOW subtab で channel/入力レベル設定 → get_info 反映。
3. ライン入力 → 受信機群が同時振動（channel 一致）。損失/遅延は移植元の `StreamStats`/`getEstimatedDelayMs` 相当で計測。
4. **実機検証完了までローカル commit で止める**（push/tag はユーザー検証後）。

## workspace 側で完了済み

- contracts 5 spec + firmware-manifest schema。
- helper: `set_espnow_channel`/`set_input_level` リレー + get_info role/transport passthrough。
- studio: role=transmitter の ESP-NOW subtab + onboarding「ライブ送信機」+ firmware library role 分類 + dev plugin に `tx` repo root 追加。

→ ファーム側がこの指示を実装すれば Studio からの書込・設定が成立する。
