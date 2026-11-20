# Smart Oscilloscope

A small two-board oscilloscope that also roasts your signal.

- **PicoSampler** (Raspberry Pi Pico 2 W) samples 0–3.3 V on A0 and streams it over UART. A pot on A1 sets the timebase. It figures out the signal type (square, sine, triangle, sawtooth, noise, …), frequency and voltages, and when the signal changes it asks OpenAI's `gpt-5-nano` for a one-line sarcastic review.
- **CrowPanelScope** (Elecrow CrowPanel 4.3" ESP32-S3) draws the trace like an analog scope and shows the roast on the bottom line.

## Wiring

| Pico | CrowPanel / part |
|---|---|
| GP0 (pin 1, UART TX) | UART1 RX (IO18) |
| GND | GND |
| GP26 / A0 (pin 31) | signal in, 0–3.3 V only |
| GP27 / A1 (pin 32) | pot wiper (pot ends on 3V3 and GND) |
| GP15 (pin 20) | jumper to A0 for a 100 Hz test square wave |

## Setup

1. Arduino IDE boards: **arduino-pico** (Raspberry Pi Pico 2W) and **esp32** by Espressif (ESP32S3 Dev Module, QSPI PSRAM, Huge APP).
2. Libraries: **ArduinoJson** 7 and **GFX Library for Arduino** 1.6.
3. Copy `PicoSampler/secrets.example.h` to `PicoSampler/secrets.h` and fill in your WiFi and OpenAI API key. `secrets.h` is git-ignored.
4. Flash `PicoSampler` to the Pico and `CrowPanelScope` to the display.

No WiFi or key? The scope still works. The bottom line just stays empty or shows the error.
