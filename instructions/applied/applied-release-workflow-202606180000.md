# 事後承認 note: GitHub Release ワークフロー新設（Studio 周辺機器ファーム配布）

- **編集元セッション**: hapbeat-studio セッション（workspace 統括 / 2026-06-18）
- **関連**: DEC-034（ノード役割 / firmware-distribution）、hapbeat-studio の firmware library
- **ステータス**: ⚠️ **未検証（ハードウェア未確認）** — レビュー + 検証後に `completed/` へ

## 背景（なぜ入れたか）

Studio（devtools.hapbeat.com）のファームウェアライブラリは、各ファーム repo の
**GitHub Release** を集約して表示する。集約は hapbeat-studio の
`scripts/aggregate-firmware-manifest.mjs`（`REPO_SHORT` でこの repo を `tx` に
マップ済）+ `.github/workflows/deploy.yml` が担う。

しかし本 repo には **release ワークフローもタグも存在せず**、GitHub Release が
1 本も無かった。そのため ESP-NOW トランスミッタ（周辺機器）ファームは Studio に
**一度もデプロイされていなかった**。同セッションで Studio 側 `deploy.yml` を
「両ファーム repo を集約する」よう修正したのに合わせ、本 repo にも device-firmware
と同型の release パイプラインを新設した。

## 入った変更（このセッションで追加）

| ファイル | 内容 |
|---|---|
| `.github/workflows/release.yml` | master push でビルド、`v*` タグ push で GitHub Release 発行 + Studio 再デプロイ dispatch。`hapbeat-device-firmware/.github/workflows/release.yml` のミラー |
| `.github/scripts/generate-manifest.mjs` | リリース artifacts から v2 `manifest.json` を生成（serial-only 形に簡約・`id=tx/<env>` / `repo=hapbeat-transmitter-firmware` / `hapbeat:false`）。`hapbeat-device-firmware/.github/scripts/generate-manifest.mjs` のミラー |

**ファームウェアのビルド（src / platformio.ini / scripts/gen_variant.py）には一切触れていない。** 追加は CI ファイル 2 本のみ。

## 設計上のポイント

- トランスミッタは **serial-only**（Wi-Fi に参加しない）→ Wi-Fi OTA 用 `firmware_app_ota.bin` は出さず、**merged `firmware_full_serial.bin` のみ**。
- merged image は **CI 内で** `esptool merge_bin` で生成（ビルド post-script は追加しない）。classic ESP32（M5Stack）なので bootloader@0x1000 + partitions@0x8000 + app@0x10000。これは device-firmware の `merge_firmware.py` が classic-ESP32 env（atom_lite_sensor）で既に出荷しているのと同じレイアウト。
- `esptool` は v4 系に pin（`>=4.5,<5`）— v5 で `merge_bin`→`merge-bin` 等の CLI 変更があるため。

## 検証状況

- ✅ `generate-manifest.mjs` を実 `dist/<env>/variant.json` で smoke-test → 正しい v2 manifest 出力を確認。
- ✅ 出力を Studio の `aggregate-firmware-manifest.mjs` に通し、`tx/esp32` / `tx/m5stack_audio_tx`（role=transmitter / board=m5stack_basic / 周辺機器 分類）に集約されることを end-to-end 確認。
- ❌ **CI 実走なし**（`pio run` → `esptool merge_bin` の実行は未確認）。
- ❌ **ハードウェア未検証**。merged `firmware_full_serial.bin` を M5Stack に Studio prod USB-Serial で書いて起動するかは未確認。

## この repo のエージェント / 人間へのアクション

**最初の `v*` タグを push する前に、必ず以下を検証すること:**

1. `release.yml` を master push（または `workflow_dispatch`）で実走させ、`esptool merge_bin` ステップが両 env で成功することを確認。
2. 生成された `<env>_firmware_full_serial.bin` を Studio（dev or prod）の USB-Serial download モードで **実 M5Stack に書き込み**、正常起動を確認。
   - 特に partition レイアウト（default.csv / OTA 有無）が device-firmware の前提
     （NVS@0x9000 / otadata@0xE000 / app@0x10000）と一致するか。Studio の split-flash は
     otadata を 0xFF erase して app@0x10000 を ota_0 として起動させる前提。
     M5Stack default partition がこれと異なるなら merge offsets / Studio 側の扱いを要調整。
3. 問題なければ `v0.1.0` タグを切って push → 自動で Release 発行 + Studio 再デプロイ。
4. **配布物 repo のため、ユーザーの動作確認後に push/tag すること**（workspace ルール）。

検証 OK なら本 note を `completed/` へ移動。問題があれば fix instruction を新規作成。
