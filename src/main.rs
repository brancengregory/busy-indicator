use std::thread::current;

use esp_idf_hal::delay::Delay;
use esp_idf_hal::gpio::{PinDriver, Pull};
use esp_idf_hal::peripherals::Peripherals;
use log::info;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Starting LEDs on XIAO ESP32C3");

    let peripherals = Peripherals::take().unwrap();
    let mut red_led = PinDriver::output(peripherals.pins.gpio3)?;
    let mut green_led = PinDriver::output(peripherals.pins.gpio4)?;
    let mut button = PinDriver::input(peripherals.pins.gpio10)?;
    button.set_pull(Pull::Up)?;

    let delay = Delay::new_default();

    red_led.set_low()?;
    green_led.set_low()?;

    let mut is_red_active = true;
    red_led.set_high()?;

    let mut last_button_state = button.is_low();

    loop {
        let current_button_state = button.is_low();

        if current_button_state && !last_button_state {
            info!("Button pressed");
            if is_red_active {
                info!("Turning green LED ON");
                red_led.set_low()?;
                green_led.set_high()?;
                is_red_active = false;
            } else {
                info!("Turning red LED ON");
                green_led.set_low()?;
                red_led.set_high()?;
                is_red_active = true;
            }
            delay.delay_ms(200); // Debounce delay
        }

        last_button_state = current_button_state;

        delay.delay_ms(10);
    }
}
