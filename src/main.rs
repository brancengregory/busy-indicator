use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_sys as _;
use log::info;

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Starting Red/Green LED Flash");

    let peripherals = Peripherals::take().unwrap();

    let mut led_red = PinDriver::output(peripherals.pins.gpio2)?;
    let mut led_green = PinDriver::output(peripherals.pins.gpio3)?;

    led_red.set_low()?;
    led_green.set_low()?;

    loop {
        info!("Turning Red LED ON");
        led_red.set_high()?;
        led_green.set_low()?;
        FreeRtos::delay_ms(1000);

        info!("Turning Green LED ON");
        led_red.set_low()?;
        led_green.set_high()?;
        FreeRtos::delay_ms(1000);
    }
}
