//! FT6336G capacitive touch controller driver (I2C 0x38).
//!
//! On the PaperMono the panel is 480x800 (portrait); the controller reports
//! coordinates in that frame directly. INT is on GPIO4 (low while touched,
//! pulses in interrupt-trigger mode); RST and VDD enable sit behind the
//! M5IOE1 (`board::epd_power_on` releases them together with the EPD).

use embedded_hal::i2c::I2c;

pub const ADDR: u8 = 0x38;

pub mod reg {
    /// Gesture ID / touch count block starts here: 0x01 gesture, 0x02 count.
    pub const TD_STATUS: u8 = 0x02;
    /// P1 XH,XL,YH,YL,WEIGHT,MISC then P2 block (6 bytes each).
    pub const P1_XH: u8 = 0x03;
    /// Interrupt mode: 0 = polling, 1 = trigger.
    pub const G_MODE: u8 = 0xA4;
    pub const CHIP_ID: u8 = 0xA3;
    pub const FIRMWARE_ID: u8 = 0xA6;
    pub const VENDOR_ID: u8 = 0xA8;
    /// Monitor-mode idle timeout etc.
    pub const CTRL: u8 = 0x86;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TouchPoint {
    pub x: u16,
    pub y: u16,
    /// 0 = press down, 1 = lift up, 2 = contact (moving/holding).
    pub event: u8,
    pub id: u8,
}

pub struct Ft6336<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C: I2c> Ft6336<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c, addr: ADDR }
    }

    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Read an arbitrary register (diagnostics).
    pub fn read_reg(&mut self, r: u8) -> Result<u8, I2C::Error> {
        let mut b = [0u8; 1];
        self.i2c.write_read(self.addr, &[r], &mut b)?;
        Ok(b[0])
    }

    pub fn chip_id(&mut self) -> Result<u8, I2C::Error> {
        let mut b = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg::CHIP_ID], &mut b)?;
        Ok(b[0])
    }

    pub fn vendor_id(&mut self) -> Result<u8, I2C::Error> {
        let mut b = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg::VENDOR_ID], &mut b)?;
        Ok(b[0])
    }

    /// Probe the chip and put it into polling (G_MODE = 0) mode.
    ///
    /// The FT6336 firmware needs a few hundred ms after reset before its ID
    /// registers read back non-zero, so retry with the supplied delay.
    pub fn init(&mut self, delay: &mut impl embedded_hal::delay::DelayNs) -> Result<u8, I2C::Error> {
        let mut id = 0;
        for _ in 0..10 {
            id = self.chip_id()?;
            if id != 0 {
                break;
            }
            delay.delay_ms(50);
        }
        self.i2c.write(self.addr, &[reg::G_MODE, 0x00])?;
        Ok(id)
    }

    /// Read up to two touch points; returns the number of active touches.
    pub fn read(&mut self, points: &mut [TouchPoint; 2]) -> Result<usize, I2C::Error> {
        let mut buf = [0u8; 13];
        self.i2c.write_read(self.addr, &[reg::TD_STATUS], &mut buf)?;
        let n = (buf[0] & 0x0F) as usize;
        if n == 0 || n > 2 {
            return Ok(0);
        }
        for (i, p) in points.iter_mut().take(n).enumerate() {
            let o = 1 + i * 6;
            p.event = buf[o] >> 6;
            p.x = (((buf[o] & 0x0F) as u16) << 8) | buf[o + 1] as u16;
            p.id = buf[o + 2] >> 4;
            p.y = (((buf[o + 2] & 0x0F) as u16) << 8) | buf[o + 3] as u16;
        }
        Ok(n)
    }
}
