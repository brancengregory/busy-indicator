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
use esp_hal::gpio::{Input, InputConfig, Io, Level, Output, OutputConfig, Pull, WakeEvent};
use esp_hal::handler;
use esp_hal::rtc_cntl::{Rtc, sleep::GpioWakeupSource};
use esp_hal::timer::timg::TimerGroup;
use esp_radio::esp_now::{BROADCAST_ADDRESS, EspNow};
use esp_radio::wifi::{ClientConfig, ModeConfig};
use heapless::Vec;
use log::info;

// Timing constants (in milliseconds)
const DEBOUNCE_MS: u64 = 50; // Increased from 30ms for better bounce suppression
const BUTTON_POLL_MS: u64 = 50;
const LONG_PRESS_THRESHOLD_MS: u64 = 1400;
const LONG_PRESS_CONFIRM_MS: u64 = 2000;
const BLINK_DURATION_MS: u64 = 100;
const PRE_SLEEP_DELAY_MS: u64 = 100; // Increased for better settling before sleep
const POST_WAKEUP_DELAY_MS: u64 = 200; // Increased for wake stabilization
const POST_WAKE_SETTLE_MS: u64 = 500; // Time to ignore spurious presses after wake
const WAKEUP_TIMEOUT_MS: u64 = 1000;
const NUM_BLINKS: usize = 3;

// WiFi/ESP-NOW constants
const ESP_NOW_CHANNEL: u8 = 11;

/// LED pair abstraction for controlling red/green LED pairs
struct LedPair<'a> {
    red: Output<'a>,
    green: Output<'a>,
}

impl<'a> LedPair<'a> {
    /// Create a new LED pair from individual GPIO outputs
    fn new(red: Output<'a>, green: Output<'a>) -> Self {
        Self { red, green }
    }

    /// Set LED color (true = red, false = green)
    fn set_color(&mut self, is_red: bool) {
        if is_red {
            let _ = self.red.set_high();
            let _ = self.green.set_low();
        } else {
            let _ = self.red.set_low();
            let _ = self.green.set_high();
        }
    }

    /// Turn both LEDs off
    fn off(&mut self) {
        let _ = self.red.set_low();
        let _ = self.green.set_low();
    }

    /// Turn both LEDs on (creates orange/amber color)
    fn both_on(&mut self) {
        let _ = self.red.set_high();
        let _ = self.green.set_high();
    }

    /// Check if currently showing red
    fn is_red(&self) -> bool {
        self.red.is_set_high() && !self.green.is_set_high()
    }
}

// Channels for inter-task communication
static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, StateEvent, 8> = Channel::new();
static EFFECT_CHANNEL: Channel<CriticalSectionRawMutex, Effect, 8> = Channel::new();
static SLEEP_CHANNEL: Channel<CriticalSectionRawMutex, SleepCommand, 2> = Channel::new();
static SEND_CHANNEL: Channel<CriticalSectionRawMutex, bool, 2> = Channel::new();

/// System state - persists across light sleep
#[derive(Debug, Clone, Copy)]
struct SystemState {
    local_red: bool,
    remote_red: bool,
    #[allow(dead_code)]
    sleeping: bool,
}

impl Default for SystemState {
    fn default() -> Self {
        Self {
            local_red: true,
            remote_red: true,
            sleeping: false,
        }
    }
}

/// Events that can change system state
#[derive(Debug, Clone, Copy)]
enum StateEvent {
    ButtonPress,
    ButtonLongPressStarted,
    ButtonLongPress,
    RemoteUpdate(bool),
    WakeUp,
}

/// Effects to be executed based on state changes
#[derive(Debug, Clone, Copy)]
enum Effect {
    SetLocalLed(bool),
    SetRemoteLed(bool),
    AllLedsOff,
    BlinkLocal(usize),  // Blink both local LEDs together (orange/amber feedback)
    BroadcastState(()), // Unit type - state value retrieved from SystemState
    RequestSleep,
}

/// Commands for sleep/wake operations
#[derive(Debug, Clone, Copy)]
enum SleepCommand {
    EnterSleep,
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[handler]
fn gpio_handler() {
    // Empty handler allows hardware to trigger Embassy executor wakeup
}

/// Pure state transition function - no side effects, easily testable
fn transition(state: SystemState, event: StateEvent) -> (SystemState, Vec<Effect, 8>) {
    let mut effects = Vec::new();

    let new_state = match event {
        StateEvent::ButtonPress => {
            let new_local = !state.local_red;
            effects.push(Effect::SetLocalLed(new_local)).ok();
            effects.push(Effect::BroadcastState(())).ok();
            SystemState {
                local_red: new_local,
                ..state
            }
        }

        StateEvent::ButtonLongPressStarted => {
            effects.push(Effect::BlinkLocal(NUM_BLINKS)).ok();
            state
        }

        StateEvent::ButtonLongPress => {
            effects.push(Effect::AllLedsOff).ok();
            effects.push(Effect::RequestSleep).ok();
            SystemState {
                sleeping: true,
                ..state
            }
        }

        StateEvent::RemoteUpdate(peer_red) => {
            effects.push(Effect::SetRemoteLed(peer_red)).ok();
            SystemState {
                remote_red: peer_red,
                ..state
            }
        }

        StateEvent::WakeUp => {
            effects.push(Effect::BlinkLocal(NUM_BLINKS)).ok();
            effects.push(Effect::SetLocalLed(state.local_red)).ok();
            effects.push(Effect::SetRemoteLed(state.remote_red)).ok();
            effects.push(Effect::BroadcastState(())).ok();
            SystemState {
                sleeping: false,
                ..state
            }
        }
    };

    (new_state, effects)
}

/// Retry LED update with logging on failure
async fn update_led_with_retry(led: &mut LedPair<'_>, is_red: bool) {
    for attempt in 0..3 {
        led.set_color(is_red);
        Timer::after(Duration::from_millis(1)).await;

        let red_high = led.red.is_set_high();
        let green_low = !led.green.is_set_high();

        if (is_red && red_high && green_low) || (!is_red && !red_high && !green_low) {
            return;
        }

        if attempt < 2 {
            log::error!("LED update failed (attempt {}), retrying...", attempt + 1);
            Timer::after(Duration::from_millis(10)).await;
        }
    }

    log::error!("LED update failed permanently after 3 attempts");
}

#[embassy_executor::task]
async fn state_manager() {
    info!("State manager started");
    let mut state = SystemState::default();
    let event_rx = EVENT_CHANNEL.receiver();
    let effect_tx = EFFECT_CHANNEL.sender();
    let sleep_tx = SLEEP_CHANNEL.sender();

    loop {
        let event = event_rx.receive().await;
        info!("Processing event: {:?}", event);

        let (new_state, effects) = transition(state, event);
        state = new_state;

        for effect in effects {
            match effect {
                Effect::RequestSleep => {
                    sleep_tx.send(SleepCommand::EnterSleep).await;
                }
                _ => {
                    effect_tx.send(effect).await;
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn led_controller(mut local: LedPair<'static>, mut remote: LedPair<'static>) {
    info!("LED controller started");
    let effect_rx = EFFECT_CHANNEL.receiver();

    loop {
        match effect_rx.receive().await {
            Effect::SetLocalLed(is_red) => {
                update_led_with_retry(&mut local, is_red).await;
            }
            Effect::SetRemoteLed(is_red) => {
                update_led_with_retry(&mut remote, is_red).await;
            }
            Effect::AllLedsOff => {
                local.off();
                remote.off();
            }
            Effect::BlinkLocal(count) => {
                // Flash both local LEDs together (orange/amber color)
                let original_red = local.is_red();
                for _ in 0..count {
                    // Both OFF
                    local.off();
                    Timer::after(Duration::from_millis(BLINK_DURATION_MS)).await;

                    // Both ON (orange/amber color)
                    local.both_on();
                    Timer::after(Duration::from_millis(BLINK_DURATION_MS)).await;
                }
                // Restore original state
                update_led_with_retry(&mut local, original_red).await;
            }
            Effect::BroadcastState(_) | Effect::RequestSleep => {
                // Handled by other tasks
            }
        }
    }
}

#[embassy_executor::task]
async fn button_task(mut button: Input<'static>, mut rtc: Rtc<'static>) {
    info!("Button task started");
    let event_tx = EVENT_CHANNEL.sender();
    let sleep_rx = SLEEP_CHANNEL.receiver();

    loop {
        // Wait for either button press or sleep command
        match select(button.wait_for_falling_edge(), sleep_rx.receive()).await {
            Either::First(_) => {
                // Button pressed
                Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;

                let mut hold_time = Duration::from_millis(DEBOUNCE_MS);
                let mut long_press_confirmed = false;
                let mut aborted_during_blink = false;

                while button.is_low() {
                    if hold_time >= Duration::from_millis(LONG_PRESS_THRESHOLD_MS) {
                        // Emit feedback event
                        event_tx.send(StateEvent::ButtonLongPressStarted).await;

                        // Wait for blink feedback to complete (600ms)
                        for _ in 0..NUM_BLINKS {
                            Timer::after(Duration::from_millis(BLINK_DURATION_MS * 2)).await;
                            hold_time += Duration::from_millis(BLINK_DURATION_MS * 2);
                            if !button.is_low() {
                                // Button released during blink - debounce the release
                                Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
                                if !button.is_low() {
                                    aborted_during_blink = true;
                                    break;
                                }
                            }
                        }

                        if !aborted_during_blink
                            && hold_time >= Duration::from_millis(LONG_PRESS_CONFIRM_MS)
                        {
                            long_press_confirmed = true;
                        }
                        break;
                    }

                    Timer::after(Duration::from_millis(BUTTON_POLL_MS)).await;
                    hold_time += Duration::from_millis(BUTTON_POLL_MS);
                }

                // Debounce button release for normal press detection
                if !long_press_confirmed && !aborted_during_blink {
                    Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
                }

                if long_press_confirmed {
                    event_tx.send(StateEvent::ButtonLongPress).await;
                } else if !aborted_during_blink
                    && hold_time < Duration::from_millis(LONG_PRESS_THRESHOLD_MS)
                {
                    event_tx.send(StateEvent::ButtonPress).await;
                }
            }
            Either::Second(SleepCommand::EnterSleep) => {
                // Perform sleep operation
                info!("Entering light sleep");

                // Wait for button release before sleeping with debounce
                if button.is_low() {
                    button.wait_for_rising_edge().await;
                    Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
                }
                Timer::after(Duration::from_millis(PRE_SLEEP_DELAY_MS)).await;

                // Configure wakeup
                if let Err(e) = button.wakeup_enable(true, WakeEvent::LowLevel) {
                    log::error!("Failed to enable button wakeup: {:?}", e);
                    event_tx.send(StateEvent::WakeUp).await;
                    continue;
                }

                let wakeup_source = GpioWakeupSource::new();
                rtc.sleep_light(&[&wakeup_source]);

                // Woke up!
                info!("Woke up from sleep");
                event_tx.send(StateEvent::WakeUp).await;

                // Wait for button release after wake with extended settling
                Timer::after(Duration::from_millis(POST_WAKEUP_DELAY_MS)).await;

                // Ensure button is released and stable before accepting new presses
                if button.is_low() {
                    // Wait for button release with timeout (safety in case button is stuck)
                    match with_timeout(
                        Duration::from_millis(WAKEUP_TIMEOUT_MS),
                        button.wait_for_rising_edge(),
                    )
                    .await
                    {
                        Ok(()) => {
                            Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
                        }
                        Err(_) => {
                            log::warn!(
                                "Button release timeout after {}ms, continuing anyway",
                                WAKEUP_TIMEOUT_MS
                            );
                        }
                    }
                }

                // Additional settling period to ignore spurious wake bounces
                Timer::after(Duration::from_millis(POST_WAKE_SETTLE_MS)).await;
                info!("Button settled after wake");
            }
        }
    }
}

#[embassy_executor::task]
async fn esp_now_task(mut esp_now: EspNow<'static>) {
    info!("ESP-NOW task started");
    let event_tx = EVENT_CHANNEL.sender();

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
                    event_tx.send(StateEvent::RemoteUpdate(peer_red)).await;
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

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    info!("System starting");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(gpio_handler);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized");

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
    esp_now.set_channel(ESP_NOW_CHANNEL).unwrap();

    let rtc = Rtc::new(peripherals.LPWR);
    info!("Radio initialized");

    // Create LED pairs
    let local_pair = LedPair::new(
        Output::new(peripherals.GPIO3, Level::High, OutputConfig::default()),
        Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default()),
    );
    let remote_pair = LedPair::new(
        Output::new(peripherals.GPIO21, Level::High, OutputConfig::default()),
        Output::new(peripherals.GPIO20, Level::Low, OutputConfig::default()),
    );
    let button = Input::new(
        peripherals.GPIO10,
        InputConfig::default().with_pull(Pull::Up),
    );

    // Spawn all tasks
    spawner.spawn(state_manager()).unwrap();
    spawner
        .spawn(led_controller(local_pair, remote_pair))
        .unwrap();
    spawner.spawn(button_task(button, rtc)).unwrap();
    spawner.spawn(esp_now_task(esp_now)).unwrap();

    info!("All tasks spawned");

    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}
