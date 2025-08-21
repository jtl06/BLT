# BLT - Better Latency Tester
The Better Latency Tester (BLT) is an end to end latency measuring device that measures system latency from input to photon.

BLT features include:
- Automatic or manual threshhold calibration
- Run batched, self-triggered tests via USB mouse HID inputs, with latency logging.
- Microphone triggered tests for testing mouse latency.

---
## Demo
WIP - todo: add a gif of the gui

## Instructions
WIP

## Architecture
[Placeholder for Block diagram]
- MCU
  - USB HID mouse
  - ADC w/ circular DMA: light sensor
  - 1Mhz TIM clock for 1us resolution
  - LPUART over ST-Link VCP: command IO and telemtry


## Timing
WIP

## Hardware
- STM32G474 (or similar device with USB FS + ADC)
- TEMT6000 Phototransistor (A photodiode should also work)
- LM393 Microphone module
- Two USB cables

## Roadmap
- Readme and License
- Example data and demo
- 3D printed enclosure for sensor
- Composite USB
- Custom PCBs

> Inspired by the NVIDIA LDAT. This project is independent and unaffiliated with NVIDIA.
