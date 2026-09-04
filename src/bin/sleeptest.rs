//! Bench: minimal light-sleep experiment (no RTOS, no radio, no board
//! bring-up) to isolate esp-hal's light sleep on this hardware.
#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use embedded_hal::delay::DelayNs;
use esp_hal::delay::Delay;
use esp_hal::main;
use esp_hal::rtc_cntl::sleep::TimerWakeupSource;
use esp_hal::rtc_cntl::Rtc;
use log::info;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let p = esp_hal::init(config);
    let mut rtc = Rtc::new(p.LPWR);
    let mut delay = Delay::new();
    info!("sleeptest: boot, reset={:?} wake={:?}", esp_hal::rtc_cntl::reset_reason(esp_hal::system::Cpu::ProCpu), esp_hal::rtc_cntl::wakeup_cause());
    // Stay awake 5 s first so the console is usable / flashable.
    for i in 0..5 {
        info!("sleeptest: awake {i}");
        delay.delay_ms(1000);
    }
    let mut n = 0u32;
    loop {
        let t0 = rtc.time_since_power_up();
        let timer = TimerWakeupSource::new(core::time::Duration::from_millis(500));
        rtc.sleep_light(&[&timer]);
        let dt = rtc.time_since_power_up() - t0;
        n += 1;
        // Awake window long enough for USB to re-enumerate and flush logs.
        delay.delay_ms(3000);
        info!("sleeptest: woke #{n}, slept {} ms, wake={:?}", dt.as_millis(), esp_hal::rtc_cntl::wakeup_cause());
    }
}
