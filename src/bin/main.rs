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

/// Configuration for button debounce and press detection timing
#[derive(Debug, Clone, Copy)]
struct ButtonConfig {
    debounce_ms: u64,
    long_press_threshold_ms: u64,
    long_press_confirm_ms: u64,
}

impl ButtonConfig {
    /// Create a new config with sensible defaults
    fn new() -> Self {
        Self {
            debounce_ms: DEBOUNCE_MS,
            long_press_threshold_ms: LONG_PRESS_THRESHOLD_MS,
            long_press_confirm_ms: LONG_PRESS_CONFIRM_MS,
        }
    }

    /// Set debounce duration
    #[allow(dead_code)]
    fn with_debounce(mut self, ms: u64) -> Self {
        self.debounce_ms = ms;
        self
    }

    /// Set long press threshold (when feedback starts)
    #[allow(dead_code)]
    fn with_long_press_threshold(mut self, ms: u64) -> Self {
        self.long_press_threshold_ms = ms;
        self
    }

    /// Set long press confirmation time (when sleep triggers)
    #[allow(dead_code)]
    fn with_long_press_confirm(mut self, ms: u64) -> Self {
        self.long_press_confirm_ms = ms;
        self
    }
}

impl Default for ButtonConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Events emitted by button handler
#[derive(Debug, Clone, Copy)]
enum ButtonEvent {
    Press,
    LongPressStarted,
    LongPress,
}

/// Button handler with debouncing and press detection
struct ButtonHandler<'a> {
    button: Input<'a>,
    config: ButtonConfig,
}

impl<'a> ButtonHandler<'a> {
    /// Create a new button handler with default config
    fn new(button: Input<'a>) -> Self {
        Self {
            button,
            config: ButtonConfig::default(),
        }
    }

    /// Create a new button handler with custom config
    #[allow(dead_code)]
    fn with_config(button: Input<'a>, config: ButtonConfig) -> Self {
        Self { button, config }
    }

    /// Check if button is currently pressed
    fn is_pressed(&self) -> bool {
        self.button.is_low()
    }

    /// Wait for button press with debounce, returns press event type
    /// For long press, returns LongPressStarted at threshold, then caller should call wait_for_long_press_confirm()
    async fn wait_for_press(&mut self) -> ButtonEvent {
        // Wait for falling edge (press)
        self.button.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(self.config.debounce_ms)).await;

        let mut hold_time = Duration::from_millis(self.config.debounce_ms);

        while self.is_pressed() {
            // Check if we've reached long press threshold
            if hold_time >= Duration::from_millis(self.config.long_press_threshold_ms) {
                return ButtonEvent::LongPressStarted;
            }

            Timer::after(Duration::from_millis(BUTTON_POLL_MS)).await;
            hold_time += Duration::from_millis(BUTTON_POLL_MS);
        }

        // Button released before threshold - debounce and return Press
        Timer::after(Duration::from_millis(self.config.debounce_ms)).await;
        ButtonEvent::Press
    }

    /// After receiving LongPressStarted, wait for confirmation or cancellation
    async fn wait_for_long_press_confirm(&mut self) -> ButtonEvent {
        let mut hold_time = Duration::from_millis(self.config.long_press_threshold_ms);

        while self.is_pressed() {
            // Check if we've reached confirmation time
            if hold_time >= Duration::from_millis(self.config.long_press_confirm_ms) {
                return ButtonEvent::LongPress;
            }

            Timer::after(Duration::from_millis(BUTTON_POLL_MS)).await;
            hold_time += Duration::from_millis(BUTTON_POLL_MS);
        }

        // Button released before confirmation
        Timer::after(Duration::from_millis(self.config.debounce_ms)).await;
        ButtonEvent::Press // Treat as normal press if released early
    }

    /// Wait for button release with debounce
    async fn wait_for_release(&mut self) {
        if self.is_pressed() {
            self.button.wait_for_rising_edge().await;
            Timer::after(Duration::from_millis(self.config.debounce_ms)).await;
        }
    }

    /// Wait for button release with timeout
    async fn wait_for_release_timeout(&mut self, timeout_ms: u64) -> Result<(), ()> {
        if !self.is_pressed() {
            return Ok(());
        }

        match with_timeout(
            Duration::from_millis(timeout_ms),
            self.button.wait_for_rising_edge(),
        )
        .await
        {
            Ok(()) => {
                Timer::after(Duration::from_millis(self.config.debounce_ms)).await;
                Ok(())
            }
            Err(_) => Err(()),
        }
    }

    /// Configure button for wakeup from sleep
    fn enable_wakeup(&mut self) -> Result<(), ()> {
        self.button
            .wakeup_enable(true, WakeEvent::LowLevel)
            .map_err(|_| ())
    }
}

// Channels for inter-task communication
static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, StateEvent, 8> = Channel::new();
static EFFECT_CHANNEL: Channel<CriticalSectionRawMutex, Effect, 8> = Channel::new();
static SLEEP_CHANNEL: Channel<CriticalSectionRawMutex, SleepCommand, 2> = Channel::new();
static SEND_CHANNEL: Channel<CriticalSectionRawMutex, bool, 2> = Channel::new();
static ESPNOW_REINIT_CHANNEL: Channel<CriticalSectionRawMutex, (), 2> = Channel::new();

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
    let broadcast_tx = SEND_CHANNEL.sender();

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
                Effect::BroadcastState(_) => {
                    // Broadcast current state to peer via ESP-NOW
                    broadcast_tx.send(state.local_red).await;
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
async fn button_task(
    mut button_handler: ButtonHandler<'static>,
    mut rtc: Rtc<'static>,
    mut wifi_controller: esp_radio::wifi::WifiController<'static>,
) {
    info!("Button task started");
    let event_tx = EVENT_CHANNEL.sender();
    let sleep_rx = SLEEP_CHANNEL.receiver();

    loop {
        // Wait for either button press or sleep command
        match select(button_handler.wait_for_press(), sleep_rx.receive()).await {
            Either::First(button_event) => {
                match button_event {
                    ButtonEvent::Press => {
                        event_tx.send(StateEvent::ButtonPress).await;
                    }
                    ButtonEvent::LongPressStarted => {
                        // Emit the feedback event to trigger blink
                        event_tx.send(StateEvent::ButtonLongPressStarted).await;
                        // Continue monitoring for confirmation
                        let confirm_event = button_handler.wait_for_long_press_confirm().await;
                        if let ButtonEvent::LongPress = confirm_event {
                            event_tx.send(StateEvent::ButtonLongPress).await;
                        }
                    }
                    ButtonEvent::LongPress => {
                        // Should not happen directly from wait_for_press anymore
                        event_tx.send(StateEvent::ButtonLongPress).await;
                    }
                }
            }
            Either::Second(SleepCommand::EnterSleep) => {
                // Perform sleep operation
                info!("Entering light sleep");

                // Wait for button release before sleeping with debounce
                button_handler.wait_for_release().await;
                Timer::after(Duration::from_millis(PRE_SLEEP_DELAY_MS)).await;

                // Stop WiFi before sleep
                info!("Stopping WiFi controller");
                if let Err(e) = wifi_controller.stop() {
                    log::error!("Failed to stop WiFi: {:?}", e);
                }

                // Configure wakeup
                if let Err(e) = button_handler.enable_wakeup() {
                    log::error!("Failed to enable button wakeup: {:?}", e);
                    // Restart WiFi if wakeup config fails
                    if let Err(e) = wifi_controller.start() {
                        log::error!("Failed to restart WiFi after wakeup config error: {:?}", e);
                    }
                    event_tx.send(StateEvent::WakeUp).await;
                    continue;
                }

                let wakeup_source = GpioWakeupSource::new();
                rtc.sleep_light(&[&wakeup_source]);

                // Woke up!
                info!("Woke up from sleep");

                // Restart WiFi after wake
                info!("Restarting WiFi controller");
                if let Err(e) = wifi_controller.start() {
                    log::error!("Failed to restart WiFi after wake: {:?}", e);
                }

                // Signal ESP-NOW task to reinitialize after WiFi restart
                info!("Signaling ESP-NOW reinitialization");
                let _ = ESPNOW_REINIT_CHANNEL.send(()).await;

                // Give WiFi and ESP-NOW time to stabilize
                Timer::after(Duration::from_millis(500)).await;

                event_tx.send(StateEvent::WakeUp).await;

                // Wait for button release after wake with extended settling
                Timer::after(Duration::from_millis(POST_WAKEUP_DELAY_MS)).await;

                // Ensure button is released and stable before accepting new presses
                if button_handler.is_pressed() {
                    // Wait for button release with timeout (safety in case button is stuck)
                    if let Err(()) = button_handler
                        .wait_for_release_timeout(WAKEUP_TIMEOUT_MS)
                        .await
                    {
                        log::warn!(
                            "Button release timeout after {}ms, continuing anyway",
                            WAKEUP_TIMEOUT_MS
                        );
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
    let reinit_rx = ESPNOW_REINIT_CHANNEL.receiver();

    loop {
        // Wait for either ESP-NOW receive, send request, or reinit signal
        match select(
            select(esp_now.receive_async(), SEND_CHANNEL.receive()),
            reinit_rx.receive(),
        )
        .await
        {
            Either::First(either) => match either {
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
                    match esp_now.send_async(&BROADCAST_ADDRESS, &[status]).await {
                        Ok(()) => {
                            info!(
                                "Broadcasted local state: {}",
                                if local_red_state { "RED" } else { "GREEN" }
                            );
                        }
                        Err(e) => {
                            log::error!("Failed to broadcast ESP-NOW message: {:?}", e);
                        }
                    }
                }
            },
            Either::Second(()) => {
                // Reinitialize ESP-NOW after WiFi restart
                info!("Reinitializing ESP-NOW after WiFi restart");
                if let Err(e) = esp_now.set_channel(ESP_NOW_CHANNEL) {
                    log::error!("Failed to reconfigure ESP-NOW channel: {:?}", e);
                } else {
                    info!("ESP-NOW channel reconfigured successfully");
                }
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

    // Create button handler with default configuration
    let button_handler = ButtonHandler::new(Input::new(
        peripherals.GPIO10,
        InputConfig::default().with_pull(Pull::Up),
    ));

    // Spawn all tasks
    spawner.spawn(state_manager()).unwrap();
    spawner
        .spawn(led_controller(local_pair, remote_pair))
        .unwrap();
    spawner
        .spawn(button_task(button_handler, rtc, wifi_controller))
        .unwrap();
    spawner.spawn(esp_now_task(esp_now)).unwrap();

    info!("All tasks spawned");

    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}
