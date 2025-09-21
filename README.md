# BLT - Better Latency Tester
The Better Latency Tester (BLT) is an open device for click to photon latency measurements in video games. It enables evaluation of game settings, monitors, and mice through reproducible, end-to-end latency data.

BLT features include:
- Automatic or manual threshold calibration
- Run batched, self-triggered tests via USB mouse HID inputs, with latency logging
- Microphone triggered tests for testing mouse latency

---
## Demo
https://github.com/user-attachments/assets/7085f5da-f47d-4a11-8dff-cdf1d9c3fb22
TEMT6000 Phototransistor Module             |  BLT Hardware
:-------------------------:|:-------------------------:
<img src="https://github.com/user-attachments/assets/5b28370e-729f-4816-abe0-36eb18c60471">   |  <img src="https://github.com/user-attachments/assets/832a1ddd-a02a-4b15-afb8-035833a1cc8b">

## Limitations
- No absolute calibration, numbers should only be used to compare relative values, as the BLT has not been calibrated against an external reference such as a high speed (1000fps+) camera.
- Sample size requirements, latency is stochastic, requires many samples to have significant results.


## Sample Data
### Vsync On vs Off vs Reflex
| Settings                     | n   | Median | Mean   | Std Dev | Min   | Max    |
|------------------------------|-----|--------|--------|---------|-------|--------|
| 60hz, 60fps, Vsync On, Reflex Off    | 100 | 74.101 | 74.507 | 5.222   | 66.134 | 93.211 |
| 60hz, 60fps, Vsync On, Reflex On   | 100 | 74.951 | 74.572 | 4.871   | 66.426 | 82.621 |
| 60hz, 64fps Vsync Off, Reflex Off | 100 | 31.338  | 31.316  | 6.524   | 18.439 | 45.391 |
| 60hz, 64fps Vsync Off, Reflex On  | 100 | 27.529  | 27.335  | 6.268 | 6.569 | 40.024 |
| 240hz, 240fps, Vsync On, Reflex Off    | 100 | 13.261 | 13.839 | 2.507   | 9.621 | 19.396 |
| 240hz, 240fps, Vsync On, Reflex On   | 100 | 13.359 | 13.898 | 2.521   | 4.972 | 20.301 |
| 240hz, 240fps Vsync Off, Reflex Off | 100 | 10.304  | 10.457  | 1.900   | 5.338 | 15.056 |
| 240hz, 240fps Vsync Off, Reflex On  | 100 | 10.599  | 10.479  | 1.967 | 6.098 | 16.648 |

Tested in Counter-Strike 2, times in ms. Low settings. See docs for raw data. Note: CS2 implements a minimum framerate cap of 64fps when not using vsync.

### Reflex, with higher GPU Load (to maximize impact of render pipeline load)
|Settings|n|Median|Mean|Std Dev|Min|Max|
|------------------------------|-----|--------|--------|---------|-------|--------|
|240hz, uncapped (~220fps), Vsync Off, Reflex Off|100|17.723|17.586|2.213|5.388|22.063|
|240hz, uncapped (~220fps),  Vsync Off, Reflex On|100|14.364|14.361|1.862|8.834|18.980|

High settings test, with greater GPU load, Reflex has a larger, more measurable improvement.  

### Mouse Testing using Mic Mode
| Settings                     | n   | Median | Mean   | Std Dev | Min   | Max    |
|------------------------------|-----|--------|--------|---------|-------|--------|
| VXE MAD R, 125hz   | 25 | 24.294  | 24.677 | 2.441   | 21.067 | 29.780 |
| VXE MAD R, 250hz   | 25 | 21.855 | 21.915 | 1.775   | 18.865 | 25.430 |
| VXE MAD R, 500hz | 25 | 21.030  | 21.324  | 1.726   | 17.996 | 24.752 |
| VXE MAD R, 1000hz  | 25 | 20.561 | 20.346 | 1.506 | 17.291 | 22.533 |
| VXE MAD R, 2000hz   | 25 | 20.566 | 20.729 | 1.588   | 17.852 | 23.397 |
| VXE MAD R, 4000hz   | 25 | 20.518 | 20.561 | 1.100   | 19.012 | 23.409 |
| VXE MAD R, 8000hz| 25 | 19.558  | 20.029  | 1.572   | 17.983 | 23.513 |
| Razer Viper Pro V2, 4000hz  | 25 | 20.146  | 20.056  | 1.386 | 17.637 | 23.038 |
| Trigger Mode, 1000hz  | 100 | 20.146  | 20.056  | 1.386 | 17.637 | 23.038 |

Tested at 240hz, Low settings, Reflex On. At these sample size, could not find significant improvement above 1khz. TODO: Larger sample sizes

## Requirements
### Hardware
- NUCLEO-G474RE (or compatible STM32 with USB FS + ADC)
- TEMT6000 phototransistor (or photodiode)
- LM393 microphone module
- 2 USB cables (one for ST-Link serial comms, one for the USB HID Device)
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

### Operation
1. Plug the USB cable from the board into your PC under test (should enumerate as a USB HID device).
2. Place the photodiode over the area that will recieve the flash. If using mic mode, place the microphone module near the mouse click.
3. Windows: launch the prebuilt GUI executable BLT-gui.exe, Linux/macOS: run the Python app directly with ```python blt_gui.py```
4. Follow GUI to Set Mode, Calibrate, and Test. Data can be exported via CSV.


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
- Linux Sidecar for data handling
- Custom PCB for manufacturing
- Switch to MCU/board with USB HS PHY support for 8khz polling rate

> Inspired by the NVIDIA LDAT. This project is independent and unaffiliated with NVIDIA.
