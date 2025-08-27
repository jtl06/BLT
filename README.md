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
WIP

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
