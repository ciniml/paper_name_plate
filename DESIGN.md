# M5Stack PaperMono ベアメタル Rust ファームウェア — 方針

最終更新: 2026-09-01

## 1. ターゲットハードウェア (M5Stack PaperMono / SKU C153)

| 項目 | 内容 |
|---|---|
| SoC | ESP32-S3R8 (Xtensa LX7 dual core, 240 MHz), 16 MB Flash, 8 MB Octal PSRAM |
| ePaper | 3.97" 480x800 4階調, コントローラ **SSD1677** (パネルネイティブは 800(source) x 480(gate) の横長。縦長表示は回転で対応) |
| タッチ | FT6336G (I2C 0x38, INT=G4, RST/VDD_EN は M5IOE1 経由) |
| NFC | **ST25R3916** (I2C 0x50, IRQ=G6, 電源イネーブルは M5IOE1 IO4)。Lite 版は非搭載 |
| LoRa | SX1262 (SPI: MOSI=G38 MISO=G40 CLK=G39 NSS=G41, BUSY=G21, IRQ=G5) — 本プロジェクトでは当面対象外 |
| 電源管理 | **M5PM1** (M5Stack 独自 PMIC, I2C 0x6E, IRQ=G1, BOOT_OUT=G0) |
| IO エキスパンダ | **M5IOE1** (M5Stack 独自, I2C 0x4F, IRQ=G7)。EPD の電源/リセット、タッチのリセット/電源、NFC電源、TF電源などを担う |
| 充電 IC | IP2315 (I2C 0x75)。**長時間バスに接続しない**（ドキュメントの注意）。触らない |
| その他 | RX8130CE RTC (0x32), BMI270 IMU (0x68), PDM マイク, ブザー(G42), ボタン A=G2 / B=G3 |

### 1.1 ピン割り当て（ePaper 関連）

| 機能 | ピン |
|---|---|
| SPI MOSI | GPIO14 |
| SPI SCLK | GPIO15 |
| CS | GPIO16 |
| D/C | GPIO17 |
| BUSY | GPIO18 (入力 pull-up, High=busy) |
| RST | M5IOE1 IO5 (0-based index 4) |
| EPD 3.3V 電源 | M5IOE1 IO3 (0-based index 2) |
| フロントライト PWM | M5PM1 GPIO3 = PWM0 |

SPI は MISO なし・4 線 (CS/DC 別線) の SPI mode 0、参照実装は 20〜40 MHz。

### 1.2 内部 I2C バス

SDA=GPIO47, SCL=GPIO48。PM1/IOE1 は 100 kHz 既定（400 kHz 切替可）、タッチ/NFC は 400 kHz。
1 本のバスに複数デバイスがぶら下がるので、ファームウェア内では共有バス抽象（`embedded-hal-bus` の `RefCellDevice` 等）を使う。

### 1.3 M5PM1 (PMIC) の要点

- レジスタマップは `m5stack/M5PM1` の `src/M5PM1.h` に準拠（主要レジスタは `src/pm1.rs` に定義）。
- **I2C アイドルスリープ**: `I2C_CFG(0x09)` の下位 4bit がスリープタイムアウト。起動時に必ず `0x00` を書き込んで無効化する（M5GFX も同じことをしている）。スリープ中は最初のトランザクションが NACK になるので、起動直後の最初のアクセスは「ダミー START（probe）→ 少し待つ → リトライ」で吸収する。
- 起動時の設定（M5GFX 準拠）: `WDT_CNT(0x0A)=0`（WDT 無効）、`PWR_CFG(0x06) |= 0x17`（LED_EN / LDO 3.3V / DCDC 5V / CHG を有効）。
- フロントライト: GPIO3 を push-pull(`GPIO_DRV 0x13` bit3=0) + FUNC0(`0x16`) の GPIO3 を `11`(PWM0) に、`PWM_FREQ(0x34/0x35)=5000Hz`、`PWM0_L/HC(0x30/0x31)` に 12bit duty（HC の bit4 が enable）。
- シャットダウン: `SYS_CMD(0x0C) = 0xA1`。

### 1.4 M5IOE1 (IO エキスパンダ) の要点

- レジスタ: `GPIO_MODE_L/H(0x03/0x04)`（1=出力）、`GPIO_OUT_L/H(0x05/0x06)`、`GPIO_IN_L/H(0x07/0x08)`、`GPIO_DRV_L/H(0x13/0x14)`（0=push-pull）、`I2C_CFG(0x23)`（下位4bit=スリープ、起動時 0 を書く）、`PWM1..4 DUTY(0x1B..0x22)`、`PWM_FREQ(0x25/0x26)`。
- ビット割り当ては **P1..P8 → L レジスタ bit0..7、P9..P14 → H レジスタ bit0..5**（ドキュメントの「PYGn」= 1-based）。
- PaperMono で使う IO: IO3=EPD_EN, IO5=EPD_RST, IO6=TP_RST, IO13=TP_EN, IO14=TF_EN, IO4=NFC_EN, IO12=PDM_EN, IO8/IO9=LED G/B(PWM2/PWM1), IO10=LoRa RST, IO2=LoRa ANT SW, IO1=TF DET, IO0(=P1?)=RTC INT, IO4=IMU INT。

  EPD 電源投入シーケンス（M5GFX 準拠）:
  1. IO3, IO5, IO6, IO13, IO14 を出力・push-pull に設定
  2. IO3=H (EPD_EN), IO13=H, IO14=H
  3. IO5, IO6 を L → 8 ms → H → 2 ms（EPD と TP のリセット）

### 1.5 SSD1677 の要点（参照: M5GFX `Panel_SSD1677`, `M5PaperMono-OTP-Demo`）

- 初期化: `0x12` SW reset → 10 ms → `0x18 0x80`(内蔵温度センサ) → `0x0C AE C7 C3 C0 40`(ブースタ) → `0x01 DF 01 02`(480 gate) → `0x3C 01`(ボーダー) 。
- RAM: `0x24`(BW / 現在フレーム) と `0x26`(RED / 前フレーム)。1 バイト = X(source) 方向 8 ピクセル、100 バイト/行 × 480 行 = 48,000 バイト/プレーン。ゲートが反転しているため、M5GFX は data entry `0x11=0x01`(X++, Y--) で Y を反転して書く。
- 更新: `0x21`(Update Ctrl1), `0x22`(Update Ctrl2), `0x20`(Master Activation) → BUSY 待ち。
  - モノクロ内蔵 LUT: フル `0x22=0xF4/0x34`, 高速部分 `0x22=0xFF`(OTP partial) / `0x1C`。
  - 4 階調: 内蔵 OTP 4 階調は `0x1A=0x5A` + `0x22=0xD7`、または独自 LUT(`0x32`, 105 バイト + 電圧 `0x03/0x04/0x2C`)。M5GFX の quality/text/fast/fastest LUT は FreeBSD ライセンスで流用可能。
- スリープ: `0x22=0x03; 0x20` で電源オフ → `0x10 0x01` deep sleep。復帰には **ハードウェアリセット (IOE1 IO5) が必須**。

## 2. ソフトウェア方針

### 2.1 ツールチェーン / クレート

- Rust: `esp` ツールチェーン（espup 導入済み, rustc 1.95 nightly ベース）。`rust-toolchain.toml` で `channel = "esp"`。
- ターゲット: `xtensa-esp32s3-none-elf`、`build-std = ["alloc", "core"]`。
- HAL: **esp-hal 1.1.x**（安定版）+ `unstable` feature（PSRAM 等に必要になる可能性があるため）。
- 周辺: `esp-println`(log 出力, USB-Serial-JTAG), `esp-backtrace`(panic), `esp-alloc`(ヒープ), `esp-bootloader-esp-idf`(app descriptor)。
- 書き込み: `espflash` (`cargo run --release`)。
- グラフィックス: **embedded-graphics 0.8** を採用。フレームバッファは 4 階調(2bit/px)を 2 プレーンで保持し、`DrawTarget` を実装する。
- 非同期: 当面は **ブロッキング**（embassy 不使用）。NFC/タッチ/ボタンのイベント処理が必要になった時点で embassy への移行を検討する。
- ドライバは外部クレート (`ssd1677` 等) に頼らず **自前実装**する。理由: 4 階調・OTP 波形・IOE1 経由のリセットなど PaperMono 固有の事情が多く、既存クレートは 800x480 4 階調 + 外部リセットの組み合わせを想定していない。

### 2.2 クレート構成

単一パッケージ（ライブラリ + バイナリ）。ライブラリ側にドライバをモジュール分割する。

```
src/
  lib.rs            # no_std ライブラリルート
  board.rs          # ピン定義、ボード初期化（PM1/IOE1 設定, EPD 電源投入）
  i2c_reg.rs        # レジスタ R/W ヘルパ（embedded-hal I2c 上）
  pm1.rs            # M5PM1 ドライバ（I2C スリープ無効化, 電源レール, フロントライト PWM, シャットダウン）
  ioe1.rs           # M5IOE1 ドライバ（GPIO 出力/入力, PWM）
  ssd1677/
    mod.rs          # コマンド定数, 低レベル送受信, 初期化, 更新モード
    lut.rs          # 4 階調 LUT（quality/text/fast/fastest）
    framebuffer.rs  # 2 プレーン 4 階調フレームバッファ + embedded-graphics DrawTarget
  bin/main.rs       # アプリ
```

ドライバは `embedded-hal 1.0` トレイト（`I2c`, `SpiBus`, `OutputPin`, `InputPin`, `DelayNs`）に対して書き、esp-hal 固有型は `board.rs` と `main.rs` に閉じ込める。

### 2.3 段階計画

1. **環境構築** — esp-generate でスケルトン生成、ビルド確認、`cargo run` で Hello world がシリアルに出ること。
2. **PMIC / IOE1** — I2C 疎通（デバイス ID 読み出し）、I2C スリープ無効化、EPD 電源投入 + リセットシーケンス。
3. **ePaper (モノクロ)** — SSD1677 初期化、全画面白/黒、embedded-graphics で文字描画 → OTP フル更新。
4. **ePaper (4 階調 / 部分更新)** — 済 (2026-09-01): 4 階調フル更新 (quality/text/fast) + fastest 差分更新（実測 95 ms/回、表示済みバッファをアプリで保持）。10 回ごとに Text 全面リフレッシュ。アイドル 60 秒で deep sleep、IOE1 リセットで復帰。
5. **フロントライト** — 済: PM1 PWM0 で点灯確認。
6. **NFC (ST25R3916, I2C)** — 済: UID 読み出し + ISO-DEP (RATS/APDU/WTX) + NDEF Type 4 読み出し実装（実データ検証は NDEF 公開デバイス待ち）。
7. **タッチ (FT6336G)** — 済 (2026-09-01): `src/ft6336.rs`、リセット後 300 ms 待ちが必須。ポーリング方式。
8. **省電力** — 済: EPD deep sleep（アイドル 60 s）、NFC フィールドのデューティサイクル（500 ms 周期、ON 80 ms）。未: ESP32 light sleep（USB ログが止まるため保留）、PM1 タイマー起床。

### 2.4 未確定事項 / リスク

- ~~M5IOE1 の「IOn」番号が 0-based か 1-based かで資料が混在~~ → 2026-09-01 実機確認済み: M5GFX のビット位置 (bit2=EPD_EN, bit4=EPD_RST, bit5=TP_RST, H側 bit4=TP_EN, bit5=TF_EN) で正しく動作。
- 表示の座標モデル（framebuffer 行 = RAM Y = 論理 x、バイト内ビット = RAM X = 論理 y、flip なし、data entry 0x03）は実機で向き・階調ともに正しいことを確認済み (2026-09-01)。
- **書き込み時の注意**: `espflash monitor` / `espflash reset` の USB-JTAG リセットは PM1 管理のダウンロードモード（`boot:0x21`）に落ち、本体ボタンを押すまで復帰しない。書き込み後はボタンで再起動し、ログは DTR/RTS を触らないシリアルリーダで読む。
- esp-hal のオクタル PSRAM 対応。フレームバッファ 2 プレーン (96 KB) + 表示済みバッファ (96 KB) は内蔵 SRAM に収まるので、当面 PSRAM は使わない。
- ~~NFC は ST25R3916 の I2C モード。公式 RFAL ライブラリ相当を Rust で書く必要があり、工数が大きい。まず ID 読み出しに絞る。~~ → 2026-09-01: `src/st25r3916.rs`（M5Unit-NFC 準拠の最小実装）で NFC-A の UID/ATQA/SAK 読み取りを実機確認済み。NDEF/ISO-DEP は未実装。
- **I2C バスは 100 kHz 固定**。PM1/IOE1 の SPD ビット(400 kHz)は常時給電のため ESP32 リセット後も残り、100 kHz アクセスが部分的に失敗する。`main.rs` の `recover_i2c_speed` が 400 kHz で SPD を戻す。`I2C_CFG` 書き込み直後は 5 ms 待つ（次トランザクションが NACK になる）。

### 2.5 ネームプレートアプリ（2026-09-02 決定）

- NFC は **カードエミュレーション**が主役: ST25R3916 をパッシブターゲットとして動かし、スマホをかざすと連絡先/URL を NDEF で渡す。~~NDEF Type 4~~ → **Type 2 (NTAG216 相当) のエミュレーション**を採用（M5 実装と同方式、チップの自動応答が使え、スマホの標準読み書きが Type 2 で動く）。2026-09-02 スマホでの URL 受け渡し実機確認済み (`src/t2t_emu.rs`)。
- 表示内容の書き換えは Type 2 の **WRITE (0xA2)**（スマホの NFC 書き込みアプリ、例: NFC Tools）で行う。2026-09-02 実機確認済み（書込→表示更新）。BLE 併用は後回し。
- 表示: 名前・肩書・所属・URL（u8g2-fonts）。2026-09-02 表示+読み取り実機確認済み。内容のフラッシュ永続化と日本語フォントは未実装（次課題）。
- 待受はターゲットモード（フィールドはスマホ側が出す）なので、リーダのデューティサイクルより低消費電力。

## 3. 参考資料

- 製品ドキュメント: https://docs.m5stack.com/en/core/PaperMono
- 回路図: https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1267/PaperMono_SCH_V0.6.2_20260522.pdf
- SSD1677 データシート: https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1267/SSD1677.pdf
- M5PM1: https://github.com/m5stack/M5PM1 / データシート https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1207/M5PM1_Datasheet_EN.pdf
- M5IOE1: https://github.com/m5stack/M5IOE1 / データシート https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1210/IO_Expander_Datasheet_EN.pdf
- M5GFX `Panel_SSD1677.cpp` (FreeBSD license): https://github.com/m5stack/M5GFX
- 工場ファーム: https://github.com/m5stack/M5PaperMono-UserDemo
- OTP 波形デモ: https://github.com/m5stack/M5PaperMono-OTP-Demo
- esp-hal: https://github.com/esp-rs/esp-hal (docs: https://docs.espressif.com/projects/rust/esp-hal/latest/)
