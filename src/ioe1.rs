//! M5IOE1 — M5Stack 14-pin I2C I/O expander driver (subset).
//!
//! Register map follows `m5stack/M5IOE1` (`src/M5IOE1.h`).
//!
//! Pin numbering: this driver uses **0-based indices 0..=13** (`Pin(0)` is the
//! chip's "IO1"/"P1"; M5Stack documentation labels them `PYG1..PYG14` and the
//! datasheet `P1..P14`).  Bits: pins 0..=7 live in the `_L` registers
//! (bit = index), pins 8..=13 in the `_H` registers (bit = index - 8).

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::i2c_reg as reg;

/// Address used when the IOE1 is a slave on an M5Stack main board (PaperMono).
pub const ADDR: u8 = 0x4F;

pub mod regs {
    pub const UID_L: u8 = 0x00;
    pub const UID_H: u8 = 0x01;
    pub const REV: u8 = 0x02;
    /// 1 = output
    pub const GPIO_MODE_L: u8 = 0x03;
    pub const GPIO_MODE_H: u8 = 0x04;
    pub const GPIO_OUT_L: u8 = 0x05;
    pub const GPIO_OUT_H: u8 = 0x06;
    pub const GPIO_IN_L: u8 = 0x07;
    pub const GPIO_IN_H: u8 = 0x08;
    pub const GPIO_PU_L: u8 = 0x09;
    pub const GPIO_PU_H: u8 = 0x0A;
    pub const GPIO_PD_L: u8 = 0x0B;
    pub const GPIO_PD_H: u8 = 0x0C;
    /// 1 = open-drain, 0 = push-pull
    pub const GPIO_DRV_L: u8 = 0x13;
    pub const GPIO_DRV_H: u8 = 0x14;
    pub const ADC_CTRL: u8 = 0x15;
    pub const ADC_DATA_L: u8 = 0x16;
    /// PWMn duty: L = [7:0], H = [7] EN | [6] POL | [3:0] duty high nibble
    pub const PWM1_DUTY_L: u8 = 0x1B;
    /// [6] INT pull, [5] WAKE, [4] SPD, [3:0] idle sleep seconds (0 = off)
    pub const I2C_CFG: u8 = 0x23;
    pub const PWM_FREQ_L: u8 = 0x25;
    pub const PWM_FREQ_H: u8 = 0x26;
}

/// 0-based pin index (0 ..= 13).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pin(pub u8);

impl Pin {
    #[inline]
    fn split(self) -> (bool, u8) {
        let i = self.0;
        debug_assert!(i < 14);
        if i < 8 { (false, 1 << i) } else { (true, 1 << (i - 8)) }
    }
}

/// PWM channel index 0..=3 (PWM1..PWM4 in the datasheet).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PwmChannel(pub u8);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    I2c(E),
    NotFound,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::I2c(e)
    }
}

pub struct Ioe1<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C: I2c> Ioe1<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c, addr: ADDR }
    }

    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Probe the device (UID read, with wake retries) and disable the I2C
    /// idle-sleep so later accesses never race a sleeping expander.
    pub fn init(&mut self, delay: &mut impl DelayNs) -> Result<u16, Error<I2C::Error>> {
        let mut uid = None;
        for _ in 0..20 {
            match reg::read_u16_le(&mut self.i2c, self.addr, regs::UID_L) {
                Ok(v) => {
                    uid = Some(v);
                    break;
                }
                Err(_) => delay.delay_ms(10),
            }
        }
        let uid = uid.ok_or(Error::NotFound)?;
        reg::write_u8(&mut self.i2c, self.addr, regs::I2C_CFG, 0x00)?;
        // The expander re-initialises its I2C block after an I2C_CFG write and
        // NACKs the next transaction if it comes too early.
        delay.delay_ms(5);
        Ok(uid)
    }

    /// Switch the IOE1's I2C interface to 400 kHz (keeps idle-sleep disabled).
    /// Change the bus frequency only after this returns.
    pub fn set_i2c_400k(&mut self, delay: &mut impl DelayNs) -> Result<(), I2C::Error> {
        reg::write_u8(&mut self.i2c, self.addr, regs::I2C_CFG, 0x10)?;
        delay.delay_ms(5);
        Ok(())
    }

    pub fn rev(&mut self) -> Result<u8, I2C::Error> {
        reg::read_u8(&mut self.i2c, self.addr, regs::REV)
    }

    fn bank(base_l: u8, base_h: u8, pin: Pin) -> (u8, u8) {
        let (hi, mask) = pin.split();
        (if hi { base_h } else { base_l }, mask)
    }

    /// Configure `pin` as push-pull output (does not change the level).
    pub fn set_output(&mut self, pin: Pin) -> Result<(), I2C::Error> {
        let (drv, m) = Self::bank(regs::GPIO_DRV_L, regs::GPIO_DRV_H, pin);
        reg::clear_bits(&mut self.i2c, self.addr, drv, m)?;
        let (mode, m) = Self::bank(regs::GPIO_MODE_L, regs::GPIO_MODE_H, pin);
        reg::set_bits(&mut self.i2c, self.addr, mode, m)
    }

    /// Configure `pin` as input (optionally with pull-up).
    pub fn set_input(&mut self, pin: Pin, pull_up: bool) -> Result<(), I2C::Error> {
        let (mode, m) = Self::bank(regs::GPIO_MODE_L, regs::GPIO_MODE_H, pin);
        reg::clear_bits(&mut self.i2c, self.addr, mode, m)?;
        let (pu, m) = Self::bank(regs::GPIO_PU_L, regs::GPIO_PU_H, pin);
        reg::update_u8(&mut self.i2c, self.addr, pu, m, if pull_up { m } else { 0 })
    }

    pub fn write(&mut self, pin: Pin, high: bool) -> Result<(), I2C::Error> {
        let (out, m) = Self::bank(regs::GPIO_OUT_L, regs::GPIO_OUT_H, pin);
        reg::update_u8(&mut self.i2c, self.addr, out, m, if high { m } else { 0 })
    }

    /// Set several pins in the same bank at once. `mask`/`value` are raw
    /// register bit masks for the `_L` (`high_bank == false`) or `_H` bank.
    pub fn write_mask(&mut self, high_bank: bool, mask: u8, value: u8) -> Result<(), I2C::Error> {
        let r = if high_bank { regs::GPIO_OUT_H } else { regs::GPIO_OUT_L };
        reg::update_u8(&mut self.i2c, self.addr, r, mask, value)
    }

    pub fn read(&mut self, pin: Pin) -> Result<bool, I2C::Error> {
        let (inr, m) = Self::bank(regs::GPIO_IN_L, regs::GPIO_IN_H, pin);
        Ok(reg::read_u8(&mut self.i2c, self.addr, inr)? & m != 0)
    }

    /// PWM base frequency in Hz shared by all four channels.
    pub fn set_pwm_frequency(&mut self, hz: u16) -> Result<(), I2C::Error> {
        reg::write(&mut self.i2c, self.addr, regs::PWM_FREQ_L, &hz.to_le_bytes())
    }

    /// 12-bit duty for channel `ch` (0..=3). `enable == false` stops the PWM.
    pub fn set_pwm_duty(&mut self, ch: PwmChannel, duty12: u16, enable: bool) -> Result<(), I2C::Error> {
        let base = regs::PWM1_DUTY_L + ch.0 * 2;
        let lo = (duty12 & 0xFF) as u8;
        let hi = ((duty12 >> 8) as u8 & 0x0F) | if enable { 0x80 } else { 0 };
        reg::write(&mut self.i2c, self.addr, base, &[lo, hi])
    }
}
