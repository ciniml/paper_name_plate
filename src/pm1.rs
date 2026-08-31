//! M5PM1 — M5Stack power-management IC driver (subset needed by PaperMono).
//!
//! Register map follows `m5stack/M5PM1` (`src/M5PM1.h`).
//!
//! Important quirks:
//! * The PM1 has an **I2C idle-sleep** (`I2C_CFG[3:0]`, seconds).  While asleep
//!   the first transaction is NACKed and only serves as a wake-up edge on SDA.
//!   [`Pm1::init`] therefore probes with retries and then disables the sleep.
//! * `PWR_CFG` and `HOLD_CFG` auto-clear on reset / download-mode events.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::i2c_reg as reg;

pub const ADDR: u8 = 0x6E;
/// Expected value of registers 0x00..=0x01 (DEVICE_ID, DEVICE_MODEL), little endian.
pub const DEVICE_ID: u16 = 0x2050;

pub mod regs {
    pub const DEVICE_ID: u8 = 0x00;
    pub const DEVICE_MODEL: u8 = 0x01;
    pub const HW_REV: u8 = 0x02;
    pub const SW_REV: u8 = 0x03;
    pub const PWR_SRC: u8 = 0x04;
    pub const WAKE_SRC: u8 = 0x05;
    /// [4] LED_EN level, [3] BOOST_EN, [2] LDO_EN(3.3V), [1] DCDC_EN(5V), [0] CHG_EN
    pub const PWR_CFG: u8 = 0x06;
    pub const HOLD_CFG: u8 = 0x07;
    pub const BATT_LVP: u8 = 0x08;
    /// [4] SPD (1=400kHz), [3:0] idle sleep timeout in seconds (0 = disabled)
    pub const I2C_CFG: u8 = 0x09;
    pub const WDT_CNT: u8 = 0x0A;
    pub const WDT_KEY: u8 = 0x0B;
    /// [7:4] must be 0xA, [1:0] 01=shutdown 10=reboot 11=download
    pub const SYS_CMD: u8 = 0x0C;
    pub const GPIO_MODE: u8 = 0x10;
    pub const GPIO_OUT: u8 = 0x11;
    pub const GPIO_IN: u8 = 0x12;
    /// [5] LED_EN drive (1=open-drain), [4:0] GPIOn drive (1=open-drain)
    pub const GPIO_DRV: u8 = 0x13;
    pub const GPIO_PUPD0: u8 = 0x14;
    pub const GPIO_PUPD1: u8 = 0x15;
    /// 2 bits per GPIO0..3: 00=GPIO 01=IRQ 10=WAKE 11=special (GPIO3: PWM0)
    pub const GPIO_FUNC0: u8 = 0x16;
    pub const GPIO_FUNC1: u8 = 0x17;
    pub const GPIO_WAKE_EN: u8 = 0x18;
    pub const GPIO_WAKE_CFG: u8 = 0x19;
    pub const VREF_L: u8 = 0x20;
    pub const VBAT_L: u8 = 0x22;
    pub const VIN_L: u8 = 0x24;
    pub const V5VINOUT_L: u8 = 0x26;
    /// PWM0 duty low byte; 0x31 = [4] enable, [3:0] duty high nibble (12-bit duty)
    pub const PWM0_L: u8 = 0x30;
    pub const PWM0_HC: u8 = 0x31;
    pub const PWM1_L: u8 = 0x32;
    pub const PWM1_HC: u8 = 0x33;
    pub const PWM_FREQ_L: u8 = 0x34;
    pub const PWM_FREQ_H: u8 = 0x35;
    pub const IRQ_STATUS1: u8 = 0x40;
    pub const IRQ_STATUS2: u8 = 0x41;
    pub const IRQ_STATUS3: u8 = 0x42;
    pub const RTC_RAM_START: u8 = 0xA0;
}

pub mod pwr_cfg {
    pub const CHG_EN: u8 = 1 << 0;
    pub const DCDC_EN: u8 = 1 << 1;
    pub const LDO_EN: u8 = 1 << 2;
    pub const BOOST_EN: u8 = 1 << 3;
    pub const LED_EN_LEVEL: u8 = 1 << 4;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    I2c(E),
    /// Device did not answer with the expected ID after several wake attempts.
    NotFound,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::I2c(e)
    }
}

pub struct Pm1<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C: I2c> Pm1<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c, addr: ADDR }
    }

    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Generate a START on the bus to pull the PM1 out of I2C idle-sleep.
    /// Errors (NACK) are expected and ignored.
    pub fn wake(&mut self) {
        let mut dummy = [0u8; 1];
        let _ = reg::read(&mut self.i2c, self.addr, regs::HW_REV, &mut dummy);
    }

    /// Probe the chip (with wake-up retries), then apply the PaperMono
    /// baseline configuration used by M5GFX / the factory firmware:
    /// * I2C idle-sleep disabled
    /// * watchdog disabled
    /// * LED_EN level / LDO 3.3V / DCDC 5V / charging enabled
    pub fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        let mut found = false;
        for _ in 0..20 {
            match reg::read_u16_le(&mut self.i2c, self.addr, regs::DEVICE_ID) {
                Ok(DEVICE_ID) => {
                    found = true;
                    break;
                }
                Ok(_) => {
                    // Wrong ID: still treat as present but keep looping a bit,
                    // some firmware revisions answer garbage right after wake.
                    delay.delay_ms(5);
                }
                Err(_) => {
                    self.wake();
                    delay.delay_ms(10);
                }
            }
        }
        if !found {
            return Err(Error::NotFound);
        }

        // Disable I2C idle sleep (write twice like the factory firmware does:
        // the first write may land while the device is half asleep).
        reg::write_u8(&mut self.i2c, self.addr, regs::I2C_CFG, 0x00)?;
        delay.delay_ms(5);
        reg::write_u8(&mut self.i2c, self.addr, regs::I2C_CFG, 0x00)?;
        delay.delay_ms(5);
        // Disable watchdog.
        reg::write_u8(&mut self.i2c, self.addr, regs::WDT_CNT, 0x00)?;
        // Power rails on.
        reg::set_bits(
            &mut self.i2c,
            self.addr,
            regs::PWR_CFG,
            pwr_cfg::CHG_EN | pwr_cfg::DCDC_EN | pwr_cfg::LDO_EN | pwr_cfg::LED_EN_LEVEL,
        )?;
        Ok(())
    }

    /// Switch the PM1's I2C interface to 400 kHz (keeps idle-sleep disabled).
    /// Change the bus frequency only after this returns.
    pub fn set_i2c_400k(&mut self, delay: &mut impl DelayNs) -> Result<(), I2C::Error> {
        reg::write_u8(&mut self.i2c, self.addr, regs::I2C_CFG, 0x10)?;
        delay.delay_ms(5);
        Ok(())
    }

    pub fn hw_rev(&mut self) -> Result<u8, I2C::Error> {
        reg::read_u8(&mut self.i2c, self.addr, regs::HW_REV)
    }

    pub fn sw_rev(&mut self) -> Result<u8, I2C::Error> {
        reg::read_u8(&mut self.i2c, self.addr, regs::SW_REV)
    }

    /// Current power source: 0 = 5VIN (USB), 1 = 5VINOUT, 2 = battery.
    pub fn power_source(&mut self) -> Result<u8, I2C::Error> {
        Ok(reg::read_u8(&mut self.i2c, self.addr, regs::PWR_SRC)? & 0x07)
    }

    /// Battery voltage in millivolts (12-bit value).
    ///
    /// Right after PM1 wake-up the ADC sometimes returns a bogus low value
    /// (tens of mV observed); retry a few times until it looks plausible.
    pub fn battery_mv(&mut self) -> Result<u16, I2C::Error> {
        let mut v = 0;
        for _ in 0..5 {
            v = reg::read_u16_le(&mut self.i2c, self.addr, regs::VBAT_L)? & 0x0FFF;
            if v >= 1000 {
                break;
            }
        }
        Ok(v)
    }

    /// VIN (USB) voltage in millivolts.
    pub fn vin_mv(&mut self) -> Result<u16, I2C::Error> {
        Ok(reg::read_u16_le(&mut self.i2c, self.addr, regs::VIN_L)? & 0x0FFF)
    }

    /// Configure GPIO3 as PWM0 output for the e-paper front light and set the
    /// PWM frequency (Hz). Call once before [`Self::set_frontlight`].
    pub fn init_frontlight(&mut self, freq_hz: u16) -> Result<(), I2C::Error> {
        // GPIO3 push-pull
        reg::clear_bits(&mut self.i2c, self.addr, regs::GPIO_DRV, 1 << 3)?;
        // GPIO3 function = 11 (PWM0)
        reg::set_bits(&mut self.i2c, self.addr, regs::GPIO_FUNC0, 0xC0)?;
        reg::write(&mut self.i2c, self.addr, regs::PWM_FREQ_L, &freq_hz.to_le_bytes())?;
        self.set_frontlight(0)
    }

    /// Front light brightness 0..=255 (gamma ~2 like M5GFX; 0 = off).
    pub fn set_frontlight(&mut self, brightness: u8) -> Result<(), I2C::Error> {
        if brightness == 0 {
            return reg::write(&mut self.i2c, self.addr, regs::PWM0_L, &[0x00, 0x00]);
        }
        let br = (brightness as u32) * (brightness as u32); // 0..65025
        let duty12 = (br >> 4) as u16; // 12-bit
        let lo = (duty12 & 0xFF) as u8;
        let hi = ((duty12 >> 8) as u8 & 0x0F) | 0x10; // bit4 = enable
        reg::write(&mut self.i2c, self.addr, regs::PWM0_L, &[lo, hi])
    }

    /// Drive the LED_EN pin (red LED on PaperMono) high/low via PWR_CFG[4].
    pub fn set_led_en(&mut self, on: bool) -> Result<(), I2C::Error> {
        reg::update_u8(
            &mut self.i2c,
            self.addr,
            regs::PWR_CFG,
            pwr_cfg::LED_EN_LEVEL,
            if on { pwr_cfg::LED_EN_LEVEL } else { 0 },
        )
    }

    /// Power the whole device off (only meaningful on battery; on USB the
    /// PM1 stays powered and may reboot immediately).
    pub fn shutdown(&mut self) -> Result<(), I2C::Error> {
        reg::write_u8(&mut self.i2c, self.addr, regs::SYS_CMD, 0xA0 | 0x01)
    }
}
