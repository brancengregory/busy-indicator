#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer, with_timeout};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Io, Level, Output, OutputConfig, Pull};
use esp_hal::handler;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::esp_now::{BROADCAST_ADDRESS, EspNow};
use esp_radio::wifi::{ClientConfig, ModeConfig};
use log::info;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

pub enum Command {
    ToggleLocal,        // Single press
    SystemOnOff,        // Long press
    RemoteUpdate(bool), // Received from ESP-NOW (true = Red, false = Green)
}

static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, Command, 4> = Channel::new();
static SEND_CHANNEL: Channel<CriticalSectionRawMutex, bool, 2> = Channel::new();

#[handler]
fn gpio_handler() {
    // This can be empty. Its presence allows the hardware
    // to trigger the Embassy executor to wake up tasks.
}

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

    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(gpio_handler);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let radio_init = alloc::boxed::Box::leak(alloc::boxed::Box::new(radio_init));
    let (mut wifi_controller, interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    wifi_controller
        .set_config(&ModeConfig::Client(ClientConfig::default()))
        .expect("Failed to set Wi-Fi config");

    wifi_controller
        .start()
        .expect("Failed to start Wi-Fi controller");

    let esp_now = interfaces.esp_now;
    esp_now.set_channel(11).unwrap();

    info!("Radio initialized!");

    let led_local_red = Output::new(peripherals.GPIO3, Level::High, OutputConfig::default());
    let led_local_green = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    let led_remote_red = Output::new(peripherals.GPIO21, Level::High, OutputConfig::default());
    let led_remote_green = Output::new(peripherals.GPIO20, Level::Low, OutputConfig::default());
    let button = Input::new(
        peripherals.GPIO10,
        InputConfig::default().with_pull(Pull::Up),
    );

    spawner
        .spawn(traffic_task(
            led_local_red,
            led_local_green,
            led_remote_red,
            led_remote_green,
        ))
        .unwrap();
    spawner.spawn(button_task(button)).unwrap();
    spawner.spawn(esp_now_task(esp_now)).unwrap();

    info!("Tasks queued");

    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples
}

#[embassy_executor::task]
async fn button_task(mut button: Input<'static>) {
    info!("Button task started");
    loop {
        button.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(20)).await;

        match with_timeout(Duration::from_millis(800), button.wait_for_rising_edge()).await {
            Err(_) => {
                info!("Long press detected");
                let _ = COMMAND_CHANNEL.send(Command::SystemOnOff).await;
                button.wait_for_rising_edge().await;
            }
            Ok(_) => {
                info!("Single boring press");
                let _ = COMMAND_CHANNEL.send(Command::ToggleLocal).await;
            }
        }
        // Debounce wait
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
async fn traffic_task(
    mut led_local_red: Output<'static>,
    mut led_local_green: Output<'static>,
    mut led_remote_red: Output<'static>,
    mut led_remote_green: Output<'static>,
) {
    info!("Traffic task started");
    let mut system_on = true;
    let mut local_is_red = true;

    loop {
        let cmd = COMMAND_CHANNEL.receive().await;

        match cmd {
            Command::SystemOnOff => {
                system_on = !system_on;
                if system_on {
                    info!("System awake");
                    set_leds(&mut led_local_red, &mut led_local_green, local_is_red);
                    let _ = SEND_CHANNEL.send(local_is_red).await;
                } else {
                    info!("System sleeping");
                    led_local_red.set_low();
                    led_local_green.set_low();
                    led_remote_red.set_low();
                    led_remote_green.set_low();
                }
            }
            Command::ToggleLocal if system_on => {
                local_is_red = !local_is_red;
                set_leds(&mut led_local_red, &mut led_local_green, local_is_red);

                // Signal esp-now task to broadcase state
                let _ = SEND_CHANNEL.send(local_is_red).await;
            }
            Command::RemoteUpdate(is_red) if system_on => {
                set_leds(&mut led_remote_red, &mut led_remote_green, is_red);
            }
            _ => {
                info!("System is asleep - ignoring command");
            }
        }
    }
}

#[embassy_executor::task]
async fn esp_now_task(mut esp_now: EspNow<'static>) {
    info!("ESP-NOW task started");
    loop {
        match select(esp_now.receive_async(), SEND_CHANNEL.receive()).await {
            Either::First(res) => {
                let data = res.data();
                if !data.is_empty() {
                    let peer_red = data[0] == 1;
                    info!(
                        "Data received from peer: {}",
                        if peer_red { "RED" } else { "GREEN" }
                    );
                    let _ = COMMAND_CHANNEL.send(Command::RemoteUpdate(peer_red)).await;
                }
            }
            Either::Second(local_red_state) => {
                let status = if local_red_state { 1 } else { 0 };
                let _ = esp_now.send_async(&BROADCAST_ADDRESS, &[status]).await;
                info!(
                    "Broadcasted local state: {}",
                    if local_red_state { "RED" } else { "GREEN" }
                );
            }
        }
    }
}

fn set_leds(red: &mut Output, green: &mut Output, is_red: bool) {
    if is_red {
        red.set_high();
        green.set_low();
    } else {
        red.set_low();
        green.set_high();
    }
}
