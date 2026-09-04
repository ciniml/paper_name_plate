//! PaperMono board definition: pin map, bring-up sequence and the [`Board`]
//! struct that owns all peripheral drivers.
//!
//! Everything esp-hal specific lives here (and in `bin/main.rs`) so the
//! drivers stay portable.

use core::cell::RefCell;

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use embedded_hal_bus::i2c::RefCellDevice;
use esp_hal::delay::Delay;
use esp_hal::gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::rtc_cntl::Rtc;
use esp_hal::i2c::master::{Config as I2cConfig, I2c as EspI2c};
use esp_hal::peripherals::Peripherals;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::time::Rate;
use esp_hal::Blocking;
use log::{error, info, warn};
use static_cell::StaticCell;

use crate::ft6336::Ft6336;
use crate::ioe1::{Ioe1, Pin as IoePin};
use crate::pm1::Pm1;
use crate::ssd1677::Ssd1677;
use crate::st25r3916::St25r3916;
use esp_storage::FlashStorage;

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


/// The shared internal I2C bus.
pub type Bus = RefCell<EspI2c<'static, Blocking>>;
/// A device handle on the shared bus.
pub type BusDevice = RefCellDevice<'static, EspI2c<'static, Blocking>>;
/// The e-paper driver with its concrete transport types.
pub type Epd = Ssd1677<Spi<'static, Blocking>, Output<'static>, Output<'static>, Input<'static>>;

static I2C_BUS: StaticCell<Bus> = StaticCell::new();

/// All PaperMono peripherals, brought up and ready to use.
pub struct Board {
    pub delay: Delay,
    pub pm1: Pm1<BusDevice>,
    pub ioe: Ioe1<BusDevice>,
    pub epd: Epd,
    /// `None` when the touch controller did not respond.
    pub touch: Option<Ft6336<BusDevice>>,
    /// `None` on the Lite model (no ST25R3916) or when init failed.
    pub nfc: Option<St25r3916<BusDevice>>,
    /// Touch INT (GPIO4, low while touched in polling mode).
    pub tp_int: Input<'static>,
    /// ST25R3916 IRQ (GPIO6, high while an unmasked interrupt is pending).
    /// Lets the app skip I2C polling of the NFC chip when nothing happened.
    pub nfc_irq: Input<'static>,
    pub button_a: Input<'static>,
    pub button_b: Input<'static>,
    /// RTC controller: deep sleep entry.
    pub rtc: Rtc<'static>,
    /// Second handles on the wake-up pins (NFC IRQ, touch INT, button A,
    /// button B) for configuring RTC-IO wake-up before deep sleep. The
    /// `Input`s above keep owning the pads at run time.
    pub wake_pins: [AnyPin<'static>; 4],
    pub flash: FlashStorage<'static>,
    /// Taken by the app to start BLE.
    pub bt: Option<esp_hal::peripherals::BT<'static>>,
}

impl Board {
    /// Bring the whole board up. Order matters:
    /// PM1 (rails, I2C sleep off) -> IOE1 -> EPD/TP power + reset -> SPI/EPD ->
    /// touch -> NFC. Failures of optional parts are logged, not fatal.
    pub fn init(p: Peripherals) -> Self {
        let mut delay = Delay::new();

        // Preemptive scheduler (required by esp-radio for BLE). The current
        // context continues as the main task.
        let timg0 = TimerGroup::new(p.TIMG0);
        let sw_int = esp_hal::interrupt::software::SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        // I2C @ 100 kHz. PM1/IOE1 default to 100 kHz and remember their SPD
        // (400 kHz) bit across ESP32 resets (the PMIC is always powered);
        // recover_i2c_speed() puts stray 400 kHz-mode devices back.
        let i2c = EspI2c::new(p.I2C0, I2cConfig::default().with_frequency(Rate::from_khz(100)))
            .expect("i2c config")
            .with_sda(p.GPIO47)
            .with_scl(p.GPIO48);
        let bus: &'static Bus = I2C_BUS.init(RefCell::new(i2c));

        // ---- PMIC ----
        let mut pm1 = Pm1::new(RefCellDevice::new(bus));
        if pm1.init(&mut delay).is_err() {
            warn!("PM1 init failed at 100 kHz; resetting PM1/IOE1 I2C speed");
            recover_i2c_speed(bus, &mut delay);
        }
        match pm1.init(&mut delay) {
            Ok(()) => {
                let hw = pm1.hw_rev().unwrap_or(0);
                let sw = pm1.sw_rev().unwrap_or(0);
                let src = pm1.power_source().unwrap_or(0xFF);
                let vbat = pm1.battery_mv().unwrap_or(0);
                let vin = pm1.vin_mv().unwrap_or(0);
                info!("PM1 ok: hw={hw} sw={sw} src={src} vbat={vbat}mV vin={vin}mV");
            }
            Err(e) => error!("PM1 init failed: {e:?}"),
        }

        // ---- IO expander + EPD/TP/NFC power and reset ----
        let mut ioe = Ioe1::new(RefCellDevice::new(bus));
        match ioe.init(&mut delay) {
            Ok(uid) => info!("IOE1 ok: uid=0x{uid:04X} rev={}", ioe.rev().unwrap_or(0)),
            Err(e) => error!("IOE1 init failed: {e:?}"),
        }
        if let Err(e) = epd_power_on(&mut ioe, &mut delay) {
            error!("EPD power-on via IOE1 failed: {e:?}");
        }
        if let Err(e) = nfc_power(&mut ioe, true) {
            error!("NFC power-on via IOE1 failed: {e:?}");
        }

        // ---- SSD1677 over SPI2 ----
        let spi = Spi::new(p.SPI2, SpiConfig::default().with_frequency(Rate::from_mhz(20)).with_mode(SpiMode::_0))
            .expect("spi config")
            .with_sck(p.GPIO15)
            .with_mosi(p.GPIO14);
        let dc = Output::new(p.GPIO17, Level::Low, OutputConfig::default());
        let cs = Output::new(p.GPIO16, Level::High, OutputConfig::default());
        let busy = Input::new(p.GPIO18, InputConfig::default().with_pull(Pull::Up));
        let mut epd = Ssd1677::new(spi, dc, cs, busy);
        match epd.init(&mut delay) {
            Ok(()) => info!("EPD init ok"),
            Err(e) => error!("EPD init failed: {e:?}"),
        }

        // ---- Touch (needs its own reset: ~300 ms boot time) ----
        // SAFETY: the clones are only used to program RTC-IO wake-up right
        // before deep sleep, when the `Input` drivers are no longer used.
        let wake_pins: [AnyPin<'static>; 4] = unsafe {
            [
                p.GPIO6.clone_unchecked().into(),
                p.GPIO4.clone_unchecked().into(),
                p.GPIO2.clone_unchecked().into(),
                p.GPIO3.clone_unchecked().into(),
            ]
        };
        let tp_int = Input::new(p.GPIO4, InputConfig::default().with_pull(Pull::Up));
        let nfc_irq = Input::new(p.GPIO6, InputConfig::default().with_pull(Pull::Down));
        if let Err(e) = touch_reset(&mut ioe, &mut delay) {
            error!("touch reset failed: {e:?}");
        }
        let mut ft = Ft6336::new(RefCellDevice::new(bus));
        let touch = match ft.init(&mut delay) {
            Ok(id) => {
                info!("Touch ok: chip=0x{id:02X} vendor=0x{:02X}", ft.vendor_id().unwrap_or(0));
                Some(ft)
            }
            Err(e) => {
                error!("Touch init failed: {e:?}");
                None
            }
        };

        // ---- NFC (Pro model only) ----
        delay.delay_ms(50);
        let mut st25 = St25r3916::new(RefCellDevice::new(bus));
        let nfc = match st25.init(&mut delay).and_then(|()| st25.configure_nfca(&mut delay)) {
            Ok(()) => {
                let (opc, aux, regd) = st25.status().unwrap_or((0, 0, 0));
                info!(
                    "NFC ok: identity=0x{:02X} op_ctrl=0x{opc:02X} aux=0x{aux:02X} reg=0x{regd:02X}",
                    st25.identity().unwrap_or(0)
                );
                Some(st25)
            }
            Err(e) => {
                warn!("NFC init failed (Lite model?): {e:?}");
                None
            }
        };

        let button_a = Input::new(p.GPIO2, InputConfig::default().with_pull(Pull::Up));
        let button_b = Input::new(p.GPIO3, InputConfig::default().with_pull(Pull::Up));

        // Everything is up: switch PM1/IOE1 to their 400 kHz mode and run the
        // whole bus at 400 kHz. NFC tag emulation needs the bandwidth (a T2T
        // READ answer must go FIFO->air within a few ms). The SPD bit persists
        // across ESP resets; the 100 kHz + recover_i2c_speed path at the top
        // of init() copes with that on the next boot.
        match (pm1.set_i2c_400k(&mut delay), ioe.set_i2c_400k(&mut delay)) {
            (Ok(()), Ok(())) => {
                let fast = I2cConfig::default().with_frequency(Rate::from_khz(400));
                match bus.borrow_mut().apply_config(&fast) {
                    Ok(()) => info!("I2C bus now 400 kHz"),
                    Err(e) => warn!("I2C 400 kHz switch failed: {e:?}"),
                }
            }
            (a, b) => warn!("keeping I2C at 100 kHz (pm1={a:?} ioe1={b:?})"),
        }

        let rtc = Rtc::new(p.LPWR);

        let flash = FlashStorage::new(p.FLASH);
        let bt = Some(p.BT);

        Board { delay, pm1, ioe, epd, touch, nfc, tp_int, nfc_irq, button_a, button_b, rtc, wake_pins, flash, bt }
    }
}

/// PM1/IOE1 left in 400 kHz mode by an earlier firmware: talk to them at
/// 400 kHz just long enough to clear their SPD bit, then return to 100 kHz.
fn recover_i2c_speed(bus: &Bus, delay: &mut Delay) {
    use crate::i2c_reg;
    let fast = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let slow = I2cConfig::default().with_frequency(Rate::from_khz(100));
    {
        let mut b = bus.borrow_mut();
        if let Err(e) = b.apply_config(&fast) {
            warn!("apply_config(400k) failed: {e:?}");
            return;
        }
        let r1 = i2c_reg::write_u8(&mut *b, crate::pm1::ADDR, crate::pm1::regs::I2C_CFG, 0x00);
        let r2 = i2c_reg::write_u8(&mut *b, crate::ioe1::ADDR, crate::ioe1::regs::I2C_CFG, 0x00);
        info!("I2C speed recovery: pm1={r1:?} ioe1={r2:?}");
        delay.delay_ms(10);
        let _ = b.apply_config(&slow);
    }
    delay.delay_ms(10);
}

/// Bring up the e-paper and touch power/reset lines through the IOE1 and
/// perform the hardware reset sequence used by M5GFX:
/// EPD_EN/TP_EN high (TF_EN stays low: the SD slot is unused and its rail
/// only costs current), then EPD_RST & TP_RST low 8 ms → high 2 ms.
pub fn epd_power_on<I: I2c>(ioe: &mut Ioe1<I>, delay: &mut impl DelayNs) -> Result<(), I::Error> {
    for p in [ioe::EPD_EN, ioe::EPD_RST, ioe::TP_RST, ioe::TP_EN, ioe::TF_EN] {
        ioe.set_output(p)?;
    }
    ioe.write(ioe::EPD_EN, true)?;
    ioe.write(ioe::TP_EN, true)?;
    ioe.write(ioe::TF_EN, false)?;
    delay.delay_ms(2);
    epd_hard_reset(ioe, delay)
}

/// Power the ST25R3916 (Pro model only) via IOE1 NFC_EN.
pub fn nfc_power<I: I2c>(ioe: &mut Ioe1<I>, on: bool) -> Result<(), I::Error> {
    ioe.set_output(ioe::NFC_EN)?;
    ioe.write(ioe::NFC_EN, on)
}

/// Reset only the touch controller with FT6336-appropriate timing.
/// Call with TP_EN already high; the controller needs ~300 ms afterwards
/// before its registers are valid.
pub fn touch_reset<I: I2c>(ioe: &mut Ioe1<I>, delay: &mut impl DelayNs) -> Result<(), I::Error> {
    ioe.set_output(ioe::TP_RST)?;
    ioe.write(ioe::TP_RST, false)?;
    delay.delay_ms(10);
    ioe.write(ioe::TP_RST, true)?;
    delay.delay_ms(300);
    Ok(())
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
