# paper_name_plate — M5Stack PaperMono bare-metal Rust firmware

Bare-metal (`no_std`) firmware for the M5Stack PaperMono (ESP32-S3R8, SSD1677 480x800 4-gray e-paper, ST25R3916 NFC) written with [esp-hal](https://github.com/esp-rs/esp-hal).
See `DESIGN.md` for the hardware notes and the plan.

## Prerequisites

- `espup install` (Xtensa Rust toolchain named `esp`) and `espflash`
- Before building/flashing: `source ~/export-esp.sh` (puts `xtensa-esp32s3-elf-gcc` on `PATH`)

## Build / flash

```sh
source ~/export-esp.sh
cargo build --release
ESPFLASH_PORT=/dev/ttyACMx cargo run --release   # flash + serial monitor
```

`.cargo/config.toml` sets a default `ESPFLASH_PORT`; override it on the command line if the board enumerates elsewhere.

## Layout

```
src/lib.rs              no_std library root
src/board.rs            PaperMono pin map, EPD power-on / reset via M5IOE1
src/i2c_reg.rs          register helpers over embedded-hal I2c
src/pm1.rs              M5PM1 PMIC (I2C sleep disable, rails, front light PWM)
src/ioe1.rs             M5IOE1 I/O expander (GPIO, PWM)
src/ssd1677/mod.rs      SSD1677 driver: init, mono/4-gray refresh, sleep
src/ssd1677/lut.rs      4-gray waveforms (from M5GFX, FreeBSD licence)
src/ssd1677/framebuffer.rs  2-plane framebuffer + embedded-graphics DrawTarget
src/bin/main.rs         application
```

## Web Bluetooth sender page

`docs/index.html` is a single-file page (Android Chrome) that sets the plate
text and sends a dithered image over BLE. Web Bluetooth needs an HTTPS
origin with the `bluetooth` permission, so it is served from GitHub Pages:
<https://www.fugafuga.org/paper_name_plate/> (GitHub Pages, user-site custom domain).

## Flashing a release

Each [release](https://github.com/ciniml/paper_name_plate/releases) ships
`paper_name_plate-merged.bin` (bootloader + partition table + app, 16 MB
flash layout) and the ELF. Flash the merged image at offset 0 with either:

```sh
espflash write-bin 0x0 paper_name_plate-merged.bin
# or
esptool.py --chip esp32s3 write_flash 0x0 paper_name_plate-merged.bin
```

Then set the plate text and image from
<https://www.fugafuga.org/paper_name_plate/> (Android Chrome).
