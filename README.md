# SENSOR_MSB


# BLE-Fall-Arrest

## SDK Installation

---

1. [Download](https://www.nordicsemi.com/Products/Development-software/nrf5-sdk/download) and extract nRF5 SDK
2. [Download](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-desktop/Download#infotabs) and Install nRF Connect on Desktop
3. [Download](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp&hl=en_IN&gl=US) and Install nRF Connect mobile app on phone
4. [Download](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) and Install GNU Arm Compilier.
5. [Download](https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools/Download) and Install nRF Command Line Tools.
6. [Download](https://github.com/xpack-dev-tools/windows-build-tools-xpack/releases/download/v4.2.1-2/xpack-windows-build-tools-4.2.1-2-win32-x64.zip) Build Tools and extract file to C:\BuildTools.
7. Download and Install VSCODE and Extensions

    - ms-vscode.cpptools
    - marus25.cortex-debug
8. Change SDK path on line number 3 in file .vscode/c_cpp_properties.json

9. Change SDK path on line number 11 and 25 in file .vscode/launch.json

10. For [Fota](https://devzone.nordicsemi.com/nordic/short-range-guides/b/software-development-kit/posts/getting-started-with-nordics-secure-dfu-bootloader)

## Fall Arrest Device

---
Purpose:
To collect data about well being and events.
To enable maintenace and warranty claims.

1. Fall Arrest Alert: based on jerk via accelerometer
2. Pull testing alert: Everytime to do pull testing before usage, capture 
3. Health alert: 24 hours packet

### Functionalities

---

1. To maintain history : not clear (current max 1000)
2. when asked , then send packets from start to end , each record after 200ms
3. Clear alerts when message is received.

### Controller

---

- nRF 52832 (arch Cortex M4)
- nRF SDK (17.0)
- SoftDevice (S132) -> Prop Firmware to run BLE stack
- BLE 5.0

#### Hardware

---

- Acclerometer
- LED (GPIO)
- Battery Circuit (ADC + GPIO)
- Optional UART (Debugging only)

### Software

---
BLE Device Name: Brand name - 3 bytes macid

- default Kare
- Name configurable by message
- Auto restart / reboot

Block Serial No:  max 12 bytes

- set by command

Threshold : configuration in mg

1. Fall arest: default 1500
2. testing: default 1200
3. freee fall: default 250

RTC : ?

Calibration mode: when received message from app, toggle the value

1. if set on: reboot
2. send raw value
3. if set off : reboot
4. send alert

Clear log
Reboot Message


set Time vaule of BLE to RTC function - add to change argument of function ' month - 1 ' 
Note - argument value month '0 to 11' 
nrf_cal_set_time(year, month - 1, day, hour, minute, second);


"getTimestamp" function give value of Timestamp 
add to change argument of function "nrf_cal_get_time_string" false 
Note - false means current timestamp and true means previous timestamps 
sprintf(ts,"%s",nrf_cal_get_time_string(false));

note -> need to only first time compile bootloader file and softdevice file with private key
if you wand recompile bootloader and softdevice you need to new private key generate


generate dfu zip file 
change SDK-id according to your SDK version 17.0.1
my case --sd-req 0xCB

nrfutil pkg generate --application nrf52832_xxaa.hex --application-version 1 --hw-version 52 --sd-req 0xCB --key-file private.key app_dfu_package.zip


Get Accelerometer data on file acc.c line no. 879
