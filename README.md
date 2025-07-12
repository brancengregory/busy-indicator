# Traffic Light Busy Indicators

## Problem

My partner and I interrupt eachother's flow. We minimize this by simply asking "Talk?", but that is:

- Still an interruption
- Sparks curiosity
- Carries social pressure to engage

## Solution

Two traffic-light style desk indicators that allow us to each set our own status and view the other's.

### Minimal Features

- LED for indicators
- Switch, button, or other setting input
- Enclosure for electronics

### Additional Features

- Connection to Google Calendar to use our work calendar meeting schedules as automated input
- Prompt guessing if, and asking confirmation for whether, we are busy
- Some kind of timer indicating how long we anticipate being in a given state
- Audio output with silent switch

## Inventory

### Hardware

- ESP32C3
  - Seeed Studio chip with low-power mode, Wifi, and Bluetooth
- RGB LEDs
- Wiring and solder
- 3D-printing capability for enclosure
  - Ideally hardware-less

### Software

- Rust program
- MQTT, HTTP, or direct TCP telemetry between nodes

## Plan

### Planning & Design
Define precise requirements, select final components, and design the overall system architecture.
1. Detailed Feature Specification
  - Define LED color mappings for each status (e.g., Red: Busy, Yellow: Wrapping Up, Green: Available, Blue: Meeting, Purple: Away).
  - Specify exact input mechanism (e.g., 3-position switch, multiple buttons, capacitive touch pad).
  - Outline user flow for manual status changes.
  - Detail Google Calendar integration requirements (which calendar to monitor, how often to check, how to handle conflicts with manual status).
  - Define logic for "prompt guessing" (e.g., 5 minutes before meeting, flash blue and await confirmation).
  - Decide on the display method for the timer (e.g., simple LED patterns, or acknowledge need for small OLED display if precise time is critical). Initial assumption: No OLED, use LED patterns.
2. Hardware Design & Component Selection
  - Select specific RGB LED model (common anode/cathode, brightness, size).
  - Determine appropriate resistors for the RGB LED based on ESP32C3 GPIO voltage and LED current requirements.
  - Finalize input mechanism choice (e.g., specific push buttons, toggle switch, or research ESP32C3 capacitive touch capabilities for "hardware-less" integration).
  - Sketch basic circuit diagram for ESP32C3, LED(s), and input(s).
  - List all necessary wiring, solder, and prototyping materials.
3. Software Architecture & API Research
  - Research Rust embedded development for ESP32C3 (e.g., esp-idf-hal, esp-idf-svc).
  - Investigate Google Calendar API for event retrieval (OAuth2 flow, necessary scopes, JSON response structure).
  - Choose inter-device communication protocol (e.g., MQTT, direct TCP/HTTP, ESP-NOW for local network). Recommendation: MQTT for robustness.
  - Define data structure for status messages exchanged between devices.
  - Outline main program loop and state machine for status management.
4. Enclosure Design
  - Measure ESP32C3 board and components for accurate enclosure sizing.
  - Design 3D model for the enclosure, including:
    - Mounting points/slots for ESP32C3 and LED.
    - Openings for LED light output.
    - Integration for chosen input mechanism (e.g., button holes, or thin areas for capacitive touch).
    - Ventilation if necessary.
    - Consider "hardware-less" aspects: snap-fit components, minimal screws.
  - Plan for cable routing (e.g., USB power).

### Hardware Prototyping & Assembly
Assemble and test the basic electronic components on a breadboard or perfboard.

1. ESP32C3 Setup & Basic LED Test
  - Install Rust toolchain for ESP32C3 development.
  - Connect ESP32C3 to power and computer.
  - Write a simple Rust "blink" program to verify ESP32C3 and development environment.
  - Connect RGB LED to ESP32C3 with current-limiting resistors.
  - Write Rust code to cycle through basic RGB colors (Red, Green, Blue) to test LED functionality.
2. Input Mechanism Integration
  - Wire selected input mechanism (buttons/switch) to ESP32C3 GPIO pins.
  - Write Rust code to read input state and print to serial monitor.
  - Test input responsiveness and debouncing.
  - (If capacitive touch) Implement and test capacitive touch sensing on ESP32C3.
3. Full Circuit Assembly
  - Assemble all components (ESP32C3, LED, input) on a breadboard or perfboard.
  - Verify all connections are correct and secure.
  - Perform a basic functional test of the combined hardware (e.g., input changes LED color).

### Firmware Development
Develop the complete Rust firmware for the ESP32C3, implementing all features.
1. Basic Device Firmware
  - Implement state machine for LED colors based on manual input (Red, Yellow, Green).
  - Refine LED brightness and color mixing for desired visual effect.
  - Implement basic power management (e.g., light sleep when idle).
2. Wi-Fi & Network Connectivity
  - Implement Rust code for Wi-Fi station mode connection to a specified SSID/password.
  - Add error handling for Wi-Fi connection failures.
  - Implement network time synchronization (NTP) for accurate timekeeping.
3. Google Calendar Integration
  - Set up Google Cloud Project and obtain OAuth2 credentials.
  - Implement OAuth2 token management (initial authorization, refresh token usage) on the ESP32C3 or via a helper script.
  - Write Rust code to make HTTP GET requests to the Google Calendar API.
  - Parse JSON responses from Google Calendar API to extract event data (start/end times, busy/free status).
  - Integrate calendar data into the device's status logic (e.g., override manual status during meetings).
4. Inter-Device Communication
  - Set up an MQTT client in Rust on the ESP32C3.
  - Implement publishing of own status to a dedicated MQTT topic (e.g., traffic_light/user1/status).
  - Implement subscribing to the other device's MQTT topic (e.g., traffic_light/user2/status).
  - Update the secondary LED (or a portion of the RGB LED) to reflect the other user's status.
5. User Interface & Logic
  - Implement the "prompt guessing" logic (e.g., check calendar, if meeting soon, flash LED and wait for input confirmation).
  - Implement the timer indication (e.g., subtle LED pulsing frequency or color shifts based on remaining time).
  - Develop robust state management to handle transitions between manual, calendar-driven, and prompted states.
  - Implement graceful handling of network disconnections and API errors.
6. Power Management
  - Optimize Rust code for low power consumption (e.g., putting Wi-Fi to sleep when not needed, deep sleep during long idle periods).
  - Test power consumption in various states.

### Enclosure Fabrication & Final Assembly
Produce the 3D-printed enclosures and integrate the electronics.
1. 3D Printing
  - Print two copies of the designed enclosure.
  - Inspect prints for quality and fit. Make adjustments to the 3D model if necessary and reprint.
2. Final Assembly
  - Mount the ESP32C3 and LED(s) securely within the enclosure.
  - Route wires neatly and ensure no shorts.
  - Secure the input mechanism (if physical) into its designated opening.
  - Close and secure the enclosure.

### Testing & Refinement
Thoroughly test the complete system and make any necessary adjustments.
1. Unit Testing
  - Test each software module (Wi-Fi, API calls, LED control, input handling) independently.
  - Verify correct state transitions in the firmware.
2. Integration Testing
  - Test Wi-Fi connectivity and Google Calendar integration on a single device.
  - Test inter-device communication between the two units.
  - Verify manual status changes correctly propagate and display.
  - Test calendar-driven status changes and overrides.
  - Test "prompt guessing" and confirmation flow.
  - Test timer indication (if implemented via LED patterns).
3. User Acceptance Testing
  - Place devices on desks and use them in a real-world scenario for a few days.
  - Gather feedback from both users on usability, clarity of indicators, and overall effectiveness.
  - Identify any bugs, unexpected behavior, or areas for improvement.
4. Documentation & Future Improvements
  - Document circuit diagram, 3D print files, and Rust code with comments.
  - List potential future enhancements (e.g., adding a small OLED screen, more complex timer display, mobile app integration via Bluetooth).

