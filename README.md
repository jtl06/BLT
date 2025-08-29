# BLT - Better Latency Tester
The Better Latency Tester (BLT) is an open device for click to photon latency measurements in video games. It enables evaluation of game settings, monitors, and mice through reproducible, end-to-end latency data.

BLT features include:
- Automatic or manual threshold calibration
- Run batched, self-triggered tests via USB mouse HID inputs, with latency logging
- Microphone triggered tests for testing mouse latency

---
## Demo
WIP - todo: add a gif of the gui

## Sample Data
| Settings                     | n   | Median | Mean   | Std Dev | Min   | Max    |
|------------------------------|-----|--------|--------|---------|-------|--------|
| 60hz, 60fps, Vsync On, Reflex Off    | 100 | 74.101 | 74.507 | 5.222   | 66.134 | 93.211 |
| 60hz, 60fps, Vsync On, Reflex On   | 100 | 74.951 | 74.572 | 4.871   | 66.426 | 82.621 |
| 60hz, 64fps Vsync Off, Reflex Off | 100 | 31.338  | 31.316  | 6.524   | 18.439 | 45.391 |
| 60hz, 64fps Vsync Off, Reflex On  | 100 | 27.529  | 27.335  | 6.268 | 6.569 | 40.024 |

Tested in Counter-Strike 2, times in ms. See docs for raw data. Note: CS2 implements a minimum framerate cap of 64fps when not using vsync

## Requirements
### Hardware
- NUCLEO-G474RE (or compatible STM32 with USB FS + ADC)
- TEMT6000 phototransistor (or photodiode)
- LM393 microphone module
- 2× USB cables (one for ST-Link, one for USB device under test)
### Software
- [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain) (arm-none-eabi-gcc, make)
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) (for .ioc regeneration)
- Python 3.9+ (for PC app)

## Instructions
### HW Pinout  
| Function                          | MCU Pin | Periph/Chan           | Notes |
|-----------------------------------|---------|------------------------|-------|
| **USB D+/D-**                        | PA12/PA11    | USB_FS                 | To PC (mouse HID signal) |
| **Light Sensor (Photodiode/PT)**  | PA0     | ADC1 INx               | Analog input |
| **Microphone (click detect)**     | PA8     | GPIO               | LM393 microphone module|
| **Serial Comms**             | PA2/PA3 | LPUART TX/RX           | Bidirectional communications for app control|
### Flashing
Flash via ST-LINK, using makefile and command ```make flash```

## Architecture
- MCU
  - USB HID mouse
  - ADC w/ circular DMA: light sensor
  - 1Mhz TIM clock for 1us resolution
  - LPUART over ST-Link VCP: command IO and telemetry


## Timing
<img width="851" height="111" alt="BLT" src="https://github.com/user-attachments/assets/40132b81-9aaa-4089-884b-2c66f89c41d9" />

Mode 2: USB Trigger, latency = t<sub>1</sub> - t<sub>0</sub>u

Mode 3: Mic Trigger, latency = t<sub>1</sub> - t<sub>0</sub>m


## Roadmap
- Finish Readme
- Example data and demo
- 3D printed enclosure for sensor
- Composite USB
- Custom PCB for manufacturing
- Switch to MCU/board with USB HS PHY support for 8khz polling rate

> Inspired by the NVIDIA LDAT. This project is independent and unaffiliated with NVIDIA.
