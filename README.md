# hapbeat-espnow-tx

M5Stack / ESP32 用の **ESP-NOW オーディオストリーミング送信機・中継器のファームウェア**。
ライン入力（またはマイク）の音声を ADPCM / Opus でエンコードし、ESP-NOW でブロードキャストする。
受信側は [hapbeat-espnow-rx](https://github.com/hapbeat/hapbeat-espnow-rx)（Arduino ライブラリ）または Hapbeat 本体。

60 台の受信機への同時ストリーミング（実運用実績あり）を、ペアリングなし・アクセスポイントなしで行える。

## ハードウェア

| 役割 | 対応機材 | env |
|---|---|---|
| 送信機 | M5Stack Basic / Core2 + [オーディオモジュール](https://www.switch-science.com/products/10417)（ES8388） | `m5stack_audio_tx` |
| 送信機 | M5Stack CoreS3 + オーディオモジュール | `m5stack_cores3_audio_tx` |
| 中継器 | M5Stack Basic / Core2（単体） | `m5stack_repeater` |
| 中継器 | Seeed XIAO ESP32-C6（単体・ヘッドレス） | `xiao_c6_repeater` |

## 書き込み

ビルド環境なしで書き込む方法が 2 つある。

1. **Web Flasher** — <https://devtools.hapbeat.com/tools/espnow-flasher/> を Chrome / Edge で開き、USB を繋いで Install を押す
2. **Hapbeat Studio** — <https://studio.hapbeat.com/> の「周辺機器」から（Hapbeat デバイスの管理と併用する場合はこちら）

自分でビルドする場合は PlatformIO で:

```bash
pio run -e m5stack_cores3_audio_tx -t upload
```

## 使い方

1. オーディオモジュールのライン入力に音源（PC のヘッドホン出力など）を接続
2. 本体の画面でモード（コーデック / レート）とチャンネル（1 / 6 / 11）を選択
3. 受信側のチャンネルを合わせる — **合っていないと一切届かない**

モードの一覧・ワイヤフォーマット・受信側の実装は
[受信ライブラリ側のドキュメント](https://github.com/hapbeat/hapbeat-espnow-rx/tree/main/docs)に集約している。

## 機能の概要

- **送信**: ES8388 からの 48 kHz 取り込み → ADPCM（低遅延系）/ Opus（低ビットレート系）エンコード → ESP-NOW ブロードキャスト。各パケットに直前フレームの複製（piggyback）を同載し、単発ロスを受信側で補完できる
- **中継器**: 受信パケットをそのまま再送して到達範囲を広げる。モードバイトの 1 ビットで中継を 1 ホップに制限するため、台数を増やしてもループしない。設定不要
- **fleet-tune（0xAC ビーコン）**: 受信機群のバッファ深さ・音量上限などを送信機から一括調整
- **シリアル設定**: Web Serial（Hapbeat Studio）から各種設定を読み書き

Hapbeat エコシステム向けの機能（再生コマンドの中継・デバイス管理連携）も同居しているが、
オーディオストリーミングの利用に必須ではない。

## ライセンス

MIT License（`LICENSE`）。依存する Opus 実装（[arduino-libopus](https://github.com/pschatzmann/arduino-libopus)）の
ライセンスは各リポジトリを参照。
