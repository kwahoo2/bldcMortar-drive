# bldcMortar-drive

The **bldcMortar** package delivers a standalone BLDC motor driver with Bluetooth Low Energy communication. It is aimed for hobbyist and educational usage.

**bldcMortar-drive** is a firmware for the [bldcMortar-hardware](https://github.com/twizzter/bldcMortar-hardware) board. This nrf52840 SoC-based board includes two DRV10983 BLDC drivers, a GPS module, two quadrature encoder inputs.

## Prerequisites

### Software Dependencies

* nRF Connect SDK
* (optional) Visual Studio Code

### Hardware

* The custom bldc-Mortar board

or

* a nRF52840-based board plus TI DRV10983 driver boards, for example [nRF52840 Dongle](https://www.nordicsemi.com/Products/Development-hardware/nrf52840-dongle) and MIKROE BRUSHLESS 3 CLICK board should work

* additionally a SWD programmer, a nRF52840 DK board can be used for this purpose. Not needed if a board with bootloader (eg. nRF52840 Dongle) is used.

## Installation

Follow the detailed [Nordic Semiconductor documentation](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/gs_programming.html) for building and programming.

Use the _mortar_board2.dts_ devicetree file for creating a build configuration.

## Using nRF52840 DK as programmer

| nRF52840 DK P20 header | bldcMortar-drive |
| -----------            | -----------                              |
| VDD nRF                | 3 (only if powered from DK!)             |
| VDD nRF                | disconnected (external power supply)     |
| SWD IO                 | IO                                       |
| SWD CLK                | CK                                       |
| GND                    | G                                        |
| RESET                  | RES (disconnected for standalone boot   )|
| VDD->SWD SEL (selects the programmer target)|                     |
|                        |UART TX->external 3V UART (logging tool)  |


![DK-Mortar][dk-mortar]

[dk-mortar]: https://raw.githubusercontent.com/kwahoo2/bldcMortar-drive/main/.github/images/dk-mortar.jpg "nRF52840 DK connected to bldcMortar-drive"

## Bluetooth connection

**bldcMortar-drive** exposes:

* 6d84a3d2-9a17-49bc-92f7-3bcc72de2137 Primary Service
* 6d84a3d2-9a17-49bc-92f7-3bcc72de2138 Characteristic, for sending commands and reading data in 8-byte messages
* 6d84a3d2-9a17-49bc-92f7-3bcc72de2139 Characteristic, for encoder 0 value, int64_t
* 6d84a3d2-9a17-49bc-92f7-3bcc72de2140 Characteristic, for encoder 1 value, int64_t


The firmware sends data to the host using notifications.

The 8-byte command (host-to-drive):

* 3 bytes as the command identifier (eg. "WRG" - "write registry")
* 1 byte as the drive id (eg. 0x01, drive 1, the second drive on the board)
* 4 bytes as the command value

### bldcMortar-tools

Use the [bldcMortar-tools](https://github.com/kwahoo2/bldcMortar-tools) to connect and drive motors. Read Texas Instruments _DRV10983 datasheet_ and _DRV10983-Q1 Tuning Guide_ to understand available options.

![Main page][mortar-page1]

[mortar-page1]: https://raw.githubusercontent.com/kwahoo2/bldcMortar-tools/main/.github/images/mortar-page1.png "Main page of the tool"

Notice logging text field in the _bldcMortar-tools_. It shows sent commands as:

    Send command: (SPF) 53 50 46 01 ff 01 00 00

The first 3 bytes 53 50 46 are the command identifier, in this case "SPF" - speed forward. 4th byte is the drive id, 0x01, the second drive. Next two 0xFF, 0x01 equals requested speed value (511, little endian). After getting the command the nRF52840 SoC writes 0x81 (0x01 | 0x80 override bit) to DRV10983 SpeeCtrl2 (0x01) registry and 0xFF to SpeeCtrl1 (0x00) registry.

### Writing characteristics directly

Command can be sent without using the _bldcMortar-tools_. Example of using **bluetoothctl** for sending the speed command from example above:

```
$ bluetoothctl
Agent registered
[bluetooth]# scan on
Discovery started
[NEW] Device 6D:8D:AE:D6:67:76 Mortar drive
[bluetooth]# scan off
[bluetooth]# connect 6D:8D:AE:D6:67:76
[Mortar drive]# menu gatt
[Mortar drive]# select-attribute 6d84a3d2-9a17-49bc-92f7-3bcc72de2138
[Mortar drive:/service0010/char0011]# write "0x53 0x50 0x46 0x01 0xFF 0x01 0x00 0x00"
```

### Storing settings in the nRF flash

By default bldcMortar-drive load registry values from the nRF52840 flash, if available. An user can save current DRV10983 registers to the flash using _bldcMortar_tools_ or writing the "WFL" command, for drive 0 and 1 respectively:

```
[Mortar drive:/service0010/char0011]# write "0x57 0x46 0x4c 0x00 0x00 0x00 0x00 0x00"
[Mortar drive:/service0010/char0011]# write "0x57 0x46 0x4c 0x01 0x00 0x00 0x00 0x00"
```

Additionally, in the _bldc_driver.c_ there are two constants defined:

```
#define DEF_MPR 0x5D //motor phase resistance
#define DEF_BEMF 0x2E //BEMF constant
```
bldcMortar-drive loads them before reading flash, so they are overwritten if the flash is not empty. You may like to adjust them to your motor parameters.

## License

Check [LICENSE](LICENSE) for details.

