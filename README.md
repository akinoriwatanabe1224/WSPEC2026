# SELECT4

## Counter Click（LS7366R）配線

GPIO番号はすべてBCM番号です。`wspec2026.kicad_sch`で確認した接続は次のとおりです。

| Raspberry Pi Zero 2 W | 物理ピン | Counter Click | LS7366Rでの役割 |
|---|---:|---|---|
| GPIO26 | 37 | RST（mikroBUS 2番） | `CNT_EN`。Highでカウント、Lowで停止 |
| GPIO27 | 13 | CS（mikroBUS 3番） | SPIチップセレクト（Active Low） |
| GPIO11 / SCLK | 23 | SCK（mikroBUS 4番） | SPIクロック |
| GPIO9 / MISO | 21 | MISO（mikroBUS 5番） | Counter ClickからRaspberry Piへのデータ |
| GPIO10 / MOSI | 19 | MOSI（mikroBUS 6番） | Raspberry PiからCounter Clickへのデータ |
| 3.3V | 1または17 | 3V3（mikroBUS 7番） | 3.3V電源 |
| GND | 6など | GND（mikroBUS 8/9番） | 共通GND |
| 5V | 2または4 | 5V（mikroBUS 10番） | Click基板へ接続される5V電源 |
| GPIO19 | 35 | INT（mikroBUS 15番） | 割り込み。現在のプログラムでは未使用 |

Counter Clickの「RST」端子は、LS7366Rではリセットではなくカウント有効入力`CNT_EN`です。GPIO26をLowにしてもカウンタ値はゼロになりません。ゼロ化にはSPIの`CLEAR_CNTR`コマンドを使用します。

### エンコーダー側

Counter Click上の2x5エンコーダー端子には、一般に次のように接続します。エンコーダーの線色や端子番号は製品ごとに異なるため、実物のデータシートで確認してください。

| エンコーダー信号 | Counter Click |
|---|---|
| A相 | ENCA |
| B相 | ENCB |
| Z相／インデックス（使用する場合） | ENCI |
| 電源 | VCC |
| 0V | GND |

Counter Clickを3.3Vで使用する場合は、基板上の`PWR SEL`を3V3側に設定します。5Vプッシュプル出力のエンコーダー信号を、3.3V動作のLS7366Rへ直接入力しないでください。エンコーダーの型番、電源電圧、出力形式（プッシュプル／オープンコレクタ）を確認し、必要ならレベル変換または3.3Vへのプルアップを使用します。

## Raspberry PiのSPI設定

SPI0を有効化します。

```bash
sudo raspi-config
```

`Interface Options` → `SPI` → `Yes`を選びます。

回路図ではCSが標準のGPIO8ではなくGPIO27に接続されているため、`/boot/firmware/config.txt`へ次を追加します。

```ini
dtoverlay=spi0-1cs,cs0_pin=27
```

他の`spi0-*cs`設定と競合させないでください。再起動後、`/dev/spidev0.0`が存在することを確認します。

```bash
ls -l /dev/spidev0.0
```

Pythonでは`spidev.open(0, 0)`を使用します。GPIO27を使う場合でも、SPIデバイス番号を27にするわけではありません。

## エンコーダー読み取り

Raspberry Pi OSでは、必要なパッケージを次のようにインストールできます。

```bash
sudo apt update
sudo apt install python3-spidev python3-gpiozero python3-lgpio
```

単体試験ではモーターを駆動せず、エンコーダーを手で回して増減を確認します。

```bash
python3 -m src.encoder.encoder --zero
```

エンコーダーの仕様が600 PPR（A相の1回転あたりパルス数）なら、4逓倍でRPMも表示できます。

```bash
python3 -m src.encoder.encoder --zero --ppr 600
```

プログラムから利用する場合は次のように読み込みます。

```python
from src.encoder.encoder import Encoder

with Encoder() as encoder:
    count = encoder.read_count()
```

初期値は回路図に合わせて、`CNT_EN=GPIO26`、SPI0、CS0（GPIO27へ割り当て）、Mode 0、500 kHz、32 bit、4逓倍です。INT（GPIO19）はポーリング読み取りでは使用しません。
