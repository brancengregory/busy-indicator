use esp_idf_hal::delay::Delay;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::peripherals::Peripherals;
use log::info;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Starting LEDs on XIAO ESP32C3");

    let peripherals = Peripherals::take().unwrap();
    let mut red_led = PinDriver::output(peripherals.pins.gpio3)?;
    let mut green_led = PinDriver::output(peripherals.pins.gpio4)?;

    let delay = Delay::new_default();

    red_led.set_low()?;
    green_led.set_low()?;

    loop {
        info!("Turning red LED ON");
        green_led.set_low()?;
        red_led.set_high()?;
        delay.delay_ms(1000);

        info!("Turning green LED ON");
        red_led.set_low()?;
        green_led.set_high()?;
        delay.delay_ms(1000);
    }
}
