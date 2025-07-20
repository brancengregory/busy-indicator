use esp_idf_hal::delay::{Delay, Ets, FreeRtos};
use esp_idf_hal::gpio::{Output, Pin, PinDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use embedded_hal::delay::DelayNs;
use log::{info};

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Starting LED test on XIAO ESP32C3");

    let peripherals = Peripherals::take().unwrap();
    let mut led = PinDriver::output(peripherals.pins.gpio2)?;

    let mut delay = Delay::new_default();

    led.set_low()?;

    loop {
        info!("Turning LED ON");
        led.set_high()?;
        delay.delay_ms(1000);
        info!("Turning LED OFF");
        led.set_low()?;
        delay.delay_ms(1000);
    }
}
