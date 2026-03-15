# Busy Indicator

A wireless status indicator system built with the XIAO ESP32C3. Designed for pairs who want to communicate their availability without interrupting each other's flow state.

## Overview

The Busy Indicator is a simple, elegant solution to a common problem: knowing when your partner or colleague is available without the social pressure of asking. Each device has two sets of red/green LEDs:

- **Local LEDs**: Show your current status (controlled by you)
- **Remote LEDs**: Show your partner's status (received wirelessly)

When your partner presses their button to change status, your remote LEDs update automatically via ESP-NOW wireless communication.

## Features

- **Dual Status Display**: Four LEDs total - two for your status, two for your partner's
- **Simple Controls**: Single button press toggles your status (red = busy, green = available)
- **Sleep Mode**: Long press (2 seconds) to enter light sleep for power saving
- **Wake on Button**: Press button to wake from sleep and resume operation
- **Wireless Sync**: ESP-NOW protocol provides instant status updates between devices
- **Visual Feedback**: Amber blinking confirms long press and wake events
- **Debounced Input**: Reliable button detection with 50ms debounce
- **Power Efficient**: Light sleep mode consumes minimal power when not in use

## Hardware

The PCB is designed around the Seeed Studio XIAO ESP32C3 with:

- **Microcontroller**: XIAO ESP32C3 (RISC-V, WiFi + Bluetooth)
- **LEDs**: 4x 3mm through-hole LEDs (2 red, 2 green)
- **Current limiting**: 4x axial resistors
- **Input**: 12mm push button
- **Power**: USB-C via XIAO board (or external battery via battery pads)

### GPIO Pinout

| Function | GPIO |
|----------|------|
| Local Red LED | GPIO3 |
| Local Green LED | GPIO4 |
| Remote Red LED | GPIO21 |
| Remote Green LED | GPIO20 |
| Button | GPIO10 |

### PCB Design Files

The hardware design is in KiCad 9.0 format:

- `hardware/busy-indicator/busy-indicator.kicad_sch` - Schematic
- `hardware/busy-indicator/busy-indicator.kicad_pcb` - PCB layout
- `hardware/busy-indicator/busy-indicator.kicad_pro` - Project settings
- `hardware/libs/XIAO_ESP32C3/` - XIAO ESP32C3 footprint and symbol library (git submodule)

## Firmware

The firmware is written in Rust using the Embassy async framework:

- **Language**: Rust (embedded no-std)
- **Framework**: Embassy (async/await for embedded)
- **HAL**: esp-hal for ESP32-C3
- **Networking**: esp-radio (ESP-NOW over WiFi)
- **Target**: riscv32imc-unknown-none-elf

### Architecture

The firmware uses an actor-model architecture with Embassy tasks:

1. **State Manager** (`state_manager`): Pure state machine handling all state transitions
2. **LED Controller** (`led_controller`): Handles LED output with retry logic
3. **Button Task** (`button_task`): Button debouncing, press detection, and sleep control
4. **ESP-NOW Task** (`esp_now_task`): Wireless communication between devices

### Key Components

- **ButtonHandler**: Debounced button input with short/long press detection
- **LedPair**: Abstraction for red/green LED pairs with color mixing (red + green = amber)
- **SystemState**: Tracks local status, remote status, and sleep state
- **Pure Transitions**: State changes are pure functions (no side effects), making testing easy

## Building

### Prerequisites

- Rust toolchain with `riscv32imc-unknown-none-elf` target
- `espflash` or `cargo-espflash` for flashing
- KiCad 9.0+ (for hardware modifications)

### Flashing Firmware

```bash
# Build and flash
cargo run --release

# Or flash pre-built binary
espflash flash target/riscv32imc-unknown-none-elf/release/busy-indicator
```

### Hardware Manufacturing

1. Open `hardware/busy-indicator/busy-indicator.kicad_pcb` in KiCad
2. Generate Gerber files for fabrication
3. Order PCBs from your preferred manufacturer (JLCPCB, PCBWay, etc.)
4. Solder components according to the schematic

## Usage

1. **Power on**: Both LED pairs will blink amber 3 times, then show red (default busy state)
2. **Toggle status**: Short press the button to switch between red (busy) and green (available)
3. **Check partner status**: Look at the "remote" LED pair - it mirrors your partner's device
4. **Enter sleep**: Long press the button (hold for 2 seconds until LEDs blink) to enter low-power mode
5. **Wake up**: Press the button to wake from sleep - LEDs will blink amber to confirm

### Wireless Pairing

Devices automatically discover each other via ESP-NOW broadcast on channel 11. No manual pairing required - just power on two devices within range and they'll sync automatically.

## Project Structure

```
busy-indicator/
├── src/
│   ├── bin/main.rs       # Firmware entry point and task definitions
│   └── lib.rs            # Library (currently empty, no_std marker)
├── hardware/
│   ├── busy-indicator/   # KiCad PCB design files
│   └── libs/             # Component libraries (XIAO_ESP32C3 submodule)
├── Cargo.toml            # Rust dependencies
├── build.rs              # Build script with helpful linker errors
└── .gitignore            # Git ignore patterns for Rust + KiCad
```

## License

This project is open source. See individual component licenses:
- Firmware: See LICENSE file in repository root
- XIAO_ESP32C3 library: See `hardware/libs/XIAO_ESP32C3/LICENSE`

## Acknowledgments

- [esp-hal](https://github.com/esp-rs/esp-hal) - Rust HAL for ESP microcontrollers
- [Embassy](https://github.com/embassy-rs/embassy) - Async/await framework for embedded
- [esp-radio](https://github.com/esp-rs/esp-radio) - WiFi/ESP-NOW support for ESP32
- [Seeed Studio](https://www.seeedstudio.com/) - XIAO ESP32C3 hardware
