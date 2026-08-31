//! PaperMono board definition: pin map and bring-up helpers.
//!
//! Everything esp-hal specific lives here so the drivers stay portable.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::ioe1::{Ioe1, Pin as IoePin};

/// M5IOE1 pins (0-based; documentation label `PYGn` == index n-1).
pub mod ioe {
    use super::IoePin;
    pub const RTC_INT: IoePin = IoePin(0);
    pub const TF_DET: IoePin = IoePin(1);
    /// 3.3 V rail of the e-paper panel.
    pub const EPD_EN: IoePin = IoePin(2);
    /// NFC (ST25R3916) power enable — factory firmware uses `M5IOE1_PIN_4`.
    pub const NFC_EN: IoePin = IoePin(3);
    /// SSD1677 RST (active low).
    pub const EPD_RST: IoePin = IoePin(4);
    /// FT6336G RST (active low).
    pub const TP_RST: IoePin = IoePin(5);
    pub const LED_G: IoePin = IoePin(7); // PWM ch 1 (index)
    pub const LED_B: IoePin = IoePin(8); // PWM ch 0 (index)
    pub const LORA_RST: IoePin = IoePin(9);
    pub const PDM_EN: IoePin = IoePin(11);
    /// FT6336G VDD enable.
    pub const TP_EN: IoePin = IoePin(12);
    /// microSD power enable.
    pub const TF_EN: IoePin = IoePin(13);
}

/// ESP32-S3 GPIO numbers (for documentation; the actual pins are taken from
/// `esp_hal::peripherals::Peripherals` in `main.rs`).
pub mod gpio {
    pub const PM1_BOOT_OUT: u8 = 0;
    pub const PM1_IRQ: u8 = 1;
    pub const BTN_A: u8 = 2;
    pub const BTN_B: u8 = 3;
    pub const TP_INT: u8 = 4;
    pub const LORA_IRQ: u8 = 5;
    pub const NFC_IRQ: u8 = 6;
    pub const IOE1_IRQ: u8 = 7;
    pub const EPD_MOSI: u8 = 14;
    pub const EPD_SCK: u8 = 15;
    pub const EPD_CS: u8 = 16;
    pub const EPD_DC: u8 = 17;
    pub const EPD_BUSY: u8 = 18;
    pub const BUZZER: u8 = 42;
    pub const I2C_SDA: u8 = 47;
    pub const I2C_SCL: u8 = 48;
}

/// I2C addresses on the internal bus (SDA 47 / SCL 48).
pub mod i2c_addr {
    pub const RTC_RX8130: u8 = 0x32;
    pub const TOUCH_FT6336: u8 = 0x38;
    pub const IOE1: u8 = 0x4F;
    pub const NFC_ST25R3916: u8 = 0x50;
    pub const IMU_BMI270: u8 = 0x68;
    pub const PM1: u8 = 0x6E;
    /// Charger; must not stay on the bus for long (docs).
    pub const CHARGER_IP2315: u8 = 0x75;
}

/// Bring up the e-paper and touch power/reset lines through the IOE1 and
/// perform the hardware reset sequence used by M5GFX:
/// EPD_EN/TP_EN/TF_EN high, then EPD_RST & TP_RST low 8 ms → high 2 ms.
pub fn epd_power_on<I: I2c>(ioe: &mut Ioe1<I>, delay: &mut impl DelayNs) -> Result<(), I::Error> {
    for p in [ioe::EPD_EN, ioe::EPD_RST, ioe::TP_RST, ioe::TP_EN, ioe::TF_EN] {
        ioe.set_output(p)?;
    }
    ioe.write(ioe::EPD_EN, true)?;
    ioe.write(ioe::TP_EN, true)?;
    ioe.write(ioe::TF_EN, true)?;
    delay.delay_ms(2);
    epd_hard_reset(ioe, delay)
}

/// Power the ST25R3916 (Pro model only) via IOE1 NFC_EN.
pub fn nfc_power<I: I2c>(ioe: &mut Ioe1<I>, on: bool) -> Result<(), I::Error> {
    ioe.set_output(ioe::NFC_EN)?;
    ioe.write(ioe::NFC_EN, on)
}

/// Pulse EPD_RST (and TP_RST) low; required to leave SSD1677 deep sleep.
pub fn epd_hard_reset<I: I2c>(ioe: &mut Ioe1<I>, delay: &mut impl DelayNs) -> Result<(), I::Error> {
    ioe.write(ioe::EPD_RST, false)?;
    ioe.write(ioe::TP_RST, false)?;
    delay.delay_ms(10);
    ioe.write(ioe::EPD_RST, true)?;
    ioe.write(ioe::TP_RST, true)?;
    delay.delay_ms(10);
    Ok(())
}
