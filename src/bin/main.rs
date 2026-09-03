#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::main;
use log::info;

use paper_name_plate::app::App;
use paper_name_plate::board::Board;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(clippy::large_stack_frames, reason = "main owns the peripherals")]
#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    // 80 MHz halves the busy-loop power draw vs. 240 MHz; peripherals (SPI,
    // I2C, RF) run from their own clocks so EPD/NFC behaviour is unchanged,
    // rendering is merely a bit slower.
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    info!("PaperMono bare-metal Rust: boot");
    let board = Board::init(peripherals);
    App::new(board).run()
}
