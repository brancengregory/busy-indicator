#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer, with_timeout};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::timer::timg::TimerGroup;
use log::info;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.2.0

    esp_println::logger::init_logger_from_env();
		info!("Logging initialized");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

		let led_red = Output::new(peripherals.GPIO3, Level::High, OutputConfig::default());
		let led_green = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
		let button = Input::new(peripherals.GPIO10, InputConfig::default().with_pull(Pull::Up));

		let _ = spawner.spawn(button_task(led_red, led_green, button));

    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples
}

#[embassy_executor::task]
async fn button_task(mut led_red: Output<'static>, mut led_green: Output<'static>, mut button: Input<'static>) {
	let mut system_on = true;

	loop {
		button.wait_for_falling_edge().await;


		match with_timeout(Duration::from_millis(800), button.wait_for_rising_edge()).await {
			Err(_) => {
				info!("Long press detected");
				system_on = !system_on;
				if system_on {
					info!("System on");
					led_red.set_high();
				} else {
					info!("System off");
					led_red.set_low();
					led_green.set_low();
				}
				button.wait_for_rising_edge().await;
			}
			Ok(_) => {
				// No long press
				// Debounce initial release
				Timer::after(Duration::from_millis(50)).await;

				match with_timeout(Duration::from_millis(250), button.wait_for_falling_edge()).await {
					Ok(_) => {
						info!("Double press detected");
						if system_on {
							info!("Secondary press");
							led_green.toggle();
						}
						button.wait_for_rising_edge().await;
					}
					Err(_) => {
						// Single press
						if system_on {
							led_red.toggle();
							info!("Single boring press");
						}
					}
				}
			}
		}

		// Debounce wait
		Timer::after(Duration::from_millis(50)).await;
	}
}

