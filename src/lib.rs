//! Bare-metal Rust support crate for the M5Stack PaperMono (ESP32-S3R8).
//!
//! Layering (see DESIGN.md):
//! - `i2c_reg`  : tiny register R/W helpers over `embedded_hal::i2c::I2c`
//! - `pm1`      : M5PM1 power-management IC (I2C 0x6E)
//! - `ioe1`     : M5IOE1 I/O expander (I2C 0x4F) — owns EPD reset/power lines
//! - `ft6336`   : FT6336G touch controller (I2C 0x38)
//! - `isodep`   : minimal ISO14443-4 initiator (RATS, I-blocks, WTX)
//! - `ndef`     : NDEF Type 4 read + record summary
//! - `t2t_emu`  : NFC Forum Type 2 tag (NTAG216) emulation on the ST25R3916
//! - `st25r3916`: ST25R3916 NFC reader (I2C 0x50), NFC-A UID reading
//! - `ssd1677`  : SSD1677 e-paper controller (800x480 native, 4-gray) + framebuffer
//! - `board`    : PaperMono pin map, bring-up sequence and `Board` (esp-hal specific)
//! - `app`      : demo application loop and UI (esp-hal specific)
#![no_std]

extern crate alloc;

pub mod app;
pub mod board;
pub mod ft6336;
pub mod i2c_reg;
pub mod ioe1;
pub mod isodep;
pub mod ndef;
pub mod pm1;
pub mod ssd1677;
pub mod st25r3916;
pub mod t2t_emu;
