# SDC26 Wireless Hacking Workshop

Hands-on workshop materials for building a simple 433 MHz replay attack with a Raspberry Pi Pico and a CC1101 radio module.

Live slides:
https://code-byter.com/sdc26_wireless_hacking/

## What this repo contains

- `docs/index.html` - workshop slides
- `blink.py` - first MicroPython sanity check, blink the Pico LED
- `cc1101_433mhz.py` - MicroPython CC1101 driver for 433.92 MHz ASK/OOK work
- `rf_hacking.py` - capture and replay workshop exercise
- `docs/assets/` - slide assets and images

## Workshop flow

1. Clone this repo or download it as ZIP.
2. Open the slides:
   https://code-byter.com/sdc26_wireless_hacking/
   Local file: `docs/index.html`
3. Connect the Pico in Thonny using `MicroPython (Raspberry Pi Pico)`.
4. Run `blink.py` to confirm your setup works.
5. Open `rf_hacking.py`.
6. Complete the sniff and replay steps from the workshop.

## Hardware

Required:

- Raspberry Pi Pico
- CC1101 module
- 433 MHz fixed-code remote
- target device, for example an RF outlet
- jumper wires
- USB cable

## Wiring

CC1101 to Pico:

- `1 - GND` -> `GND (pin 38)`
- `2 - VCC/VDD` -> `3.3V (pin 36)`
- `3 - GDO0` -> `GP3`
- `4 - CSN/CS` -> `GP5`
- `5 - SCK/SCLK` -> `GP6`
- `6 - MOSI/SI` -> `GP7`
- `7 - MISO/SO` -> `GP4`

Important:

- Use `3.3V` only.
- The CC1101 is not 5V tolerant.

## Files in more detail

### `blink.py`

Very small first test:

- toggles the onboard LED
- prints `Blink!` in the Thonny shell

### `cc1101_433mhz.py`

Provides the CC1101 driver and radio helpers:

- SPI communication with the CC1101
- 433.92 MHz ASK/OOK configuration
- transmit helpers for replaying fixed-code signals

### `rf_hacking.py`

Main workshop exercise:

- captures a fixed-code remote transmission
- decodes address and button values
- replays the captured code

## Running the exercise

In Thonny:

1. Connect to the Pico with the MicroPython interpreter selected.
2. Run `blink.py`.
3. Open `rf_hacking.py`.
4. Uncomment or complete the sniff step.
5. Press the remote once during capture.
6. Copy the captured values into the replay step.
7. Run again and verify the device responds.

## Ethics

Use this material only on hardware you own or have explicit permission to test.

This workshop is for understanding how insecure fixed-code RF systems work so we can build better and safer systems.

## Author

code_byter

- Website: https://code-byter.com
- GitHub: https://github.com/code-byter
- Instagram: `@code_byter`
- Email: `hello@code-byter.com`
