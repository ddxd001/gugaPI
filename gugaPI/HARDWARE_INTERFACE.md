# gugaPI Hardware Interface Table

Fill or verify this table from the schematic before editing SysConfig or
firmware pin definitions.

## MCU

| Item | Value | Notes |
| --- | --- | --- |
| Board name | gugaPI | `BOARD_NAME` |
| MCU part number | MSPM0G3519 | `CONFIG_MSPM0G3519` |
| Package | LQFP-100(PZ) | From `empty_cpp.syscfg` |
| Logic voltage | 3.3V | Verify from schematic |
| Board revision | - | Not specified |
| SysConfig file | `empty_cpp.syscfg` | Source of pin mux assignments |

## Wiring Overview

```text
                         +----------------------+
                         |        gugaPI        |
                         |     MSPM0G3519       |
                         |    LQFP-100(PZ)      |
                         |                      |
PC2 / I2C2_SCL  -------->| FRAM_I2C SCL         |---- FM24CL64B SCL
PC3 / I2C2_SDA  <------->| FRAM_I2C SDA         |---- FM24CL64B SDA
                         |                      |
PA11 / I2C1_SCL -------->| INA219_I2C SCL       |---- INA219 SCL
PA10 / I2C1_SDA <------->| INA219_I2C SDA       |---- INA219 SDA
                         |                      |
PB0 / UART0_TX  -------->| LORA_UART TX         |---- LoRa RX
PB1 / UART0_RX  <--------| LORA_UART RX         |---- LoRa TX
                         |                      |
PA8 / UART1_TX  -------->| MOTOR_UART TX        |---- MotorDriver RX
PA9 / UART1_RX  <--------| MOTOR_UART RX        |---- MotorDriver TX
                         |                      |
PA1 / UART5_TX  -------->| DEBUG_UART TX        |---- USB-UART RX / PC RX
PA0 / UART5_RX  <--------| DEBUG_UART RX        |---- USB-UART TX / PC TX
                         |                      |
PB6              --------| STATUS_LED           |---- Active-low LED
PA12             --------| BUZZER               |---- Active-high buzzer
PC9              <-------- BUTTON1              |---- Active-low key
PB20             <-------- BUTTON2              |---- Active-low key
PB23             <-------- BUTTON3              |---- Active-low key
                         |                      |
PA19             <------>| SWDIO                |---- XDS110 / SWD debugger
PA20             <-------| SWCLK                |---- XDS110 / SWD debugger
                         +----------------------+

All external UART/I2C modules must share GND with gugaPI.
```

## Power And Ground

| Net | External Connection | Notes |
| --- | --- | --- |
| 3.3V | MCU logic, pull-ups, low-voltage peripherals | Verify current budget from schematic |
| GND | All external modules | UART and I2C links require common ground |
| External module power | LoRa, MotorDriver, INA219 sense side, FRAM | Voltage and current depend on selected module |

## Debug UART

| Signal | MCU Pin | MCU Peripheral | External Connection | Direction | Pull-up / Pull-down | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| DEBUG_UART_TX | PA1 | UART5_TX | USB-UART RX / PC RX | MCU output | N/A | 115200 8N1 |
| DEBUG_UART_RX | PA0 | UART5_RX | USB-UART TX / PC TX | MCU input | Not configured in board code | 115200 8N1 |

UART configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Enabled | Yes | `FEATURE_ENABLE_DEBUG_UART` |
| Use case | Log output and shell input | `FEATURE_ENABLE_LOG`, `FEATURE_ENABLE_SHELL` |
| Electrical interface | 3.3V TTL UART | Do not connect directly to RS-232 levels |
| Baud rate | 115200 | From SysConfig and board config |
| Data bits | 8 | Standard 8N1 |
| Parity | None | Standard 8N1 |
| Stop bits | 1 | Standard 8N1 |
| Flow control | None | TX/RX only |

## LoRa UART Interface

| Signal | MCU Pin | MCU Peripheral | External Connection | Direction | Pull-up / Pull-down | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| LORA_UART_TX | PB0 | UART0_TX | LoRa module RX | MCU output | N/A | Board comments identify this as package pin 20 |
| LORA_UART_RX | PB1 | UART0_RX | LoRa module TX | MCU input | Internal pull-up enabled in `Board_LoraInit()` | Board comments identify this as package pin 21 |
| LORA_GND | GND | N/A | LoRa module GND | N/A | N/A | Common ground required |

LoRa UART configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Enabled | Yes | `FEATURE_ENABLE_LORA` |
| Use case | Transparent UART pass-through | Firmware does not parse module protocol |
| Electrical interface | 3.3V TTL UART | Verify LoRa module IO voltage |
| Baud rate | 115200 | `BOARD_LORA_BAUDRATE` |
| Data bits | 8 | Standard 8N1 |
| Parity | None | Standard 8N1 |
| Stop bits | 1 | Standard 8N1 |
| Flow control | None | TX/RX only |

## MotorDriver UART Interface

| Signal | MCU Pin | MCU Peripheral | External Connection | Direction | Pull-up / Pull-down | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| MOTOR_UART_TX | PA8 | UART1_TX | MotorDriver RX | MCU output | N/A | gugaPI sends commands to MotorDriver |
| MOTOR_UART_RX | PA9 | UART1_RX | MotorDriver TX | MCU input | Internal pull-up enabled in `Board_MotorDriverInit()` | gugaPI receives replies from MotorDriver |
| MOTOR_GND | GND | N/A | MotorDriver GND | N/A | N/A | Common ground required |

MotorDriver UART configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Enabled | Yes | `FEATURE_ENABLE_MOTOR_DRIVER` |
| Use case | MotorDriver control and raw UART test | Shell commands use the binary MotorDriver frame |
| Electrical interface | 3.3V TTL UART | Direct MCU-to-MotorDriver UART |
| Baud rate | 115200 | `BOARD_MOTOR_DRIVER_BAUDRATE` |
| Data bits | 8 | Standard 8N1 |
| Parity | None | Standard 8N1 |
| Stop bits | 1 | Standard 8N1 |
| Flow control | None | TX/RX only |
| Protocol framing | Binary register frame | `SOF CMD REG LEN DATA... CRC8` |

MotorDriver frame format:

| Byte Field | Size | Value / Meaning | Notes |
| --- | --- | --- | --- |
| SOF | 1 byte | 0xAA | Start of frame |
| CMD | 1 byte | 0x01 read, 0x02 write, 0x03 heartbeat | Response ORs this with 0x80 |
| REG | 1 byte | 0x00 - 0xFF | MotorDriver register address |
| LEN | 1 byte | 0 - 32 | Read length or write payload length |
| DATA | 0 - 32 bytes | Payload | Present for write requests and data responses |
| CRC8 | 1 byte | CRC-8 poly 0x07, init 0x00 | Covers SOF through DATA |

## FRAM I2C Interface

| Signal | MCU Pin | MCU Peripheral | External Connection | Direction | Pull-up | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| FRAM_I2C_SCL | PC2 | I2C2_SCL | FM24CL64B SCL | Open-drain bus | External pull-up required by hardware; firmware recovery uses GPIO mode | `PROJECT_GUIDE.md` identifies package pin 65 |
| FRAM_I2C_SDA | PC3 | I2C2_SDA | FM24CL64B SDA | Open-drain bus | External pull-up required by hardware; firmware recovery uses GPIO mode | `PROJECT_GUIDE.md` identifies package pin 66 |
| FRAM_VCC | 3.3V | N/A | FM24CL64B VCC | N/A | N/A | Verify with selected FRAM variant |
| FRAM_GND | GND | N/A | FM24CL64B GND | N/A | N/A | Common ground |

FRAM configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Enabled | Yes | `FEATURE_ENABLE_FRAM` |
| Device | FM24CL64B-GTR | 64 Kbit / 8 KiB I2C FRAM |
| I2C instance | I2C2 | `FRAM_I2C_INST` |
| Bus speed | 400 kHz | `FRAM_I2C_BUS_SPEED_HZ` |
| 7-bit address | 0x50 | `BOARD_FRAM_I2C_ADDRESS` |
| Address pins | A2/A1/A0 hardware-dependent | Update address if strapped differently |
| Self-test address | 0x1FF0 | `BOARD_FRAM_SELF_TEST_ADDRESS` |

## INA219 I2C Interface

| Signal | MCU Pin | MCU Peripheral | External Connection | Direction | Pull-up | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| INA219_I2C_SCL | PA11 | I2C1_SCL | INA219 SCL | Open-drain bus | Internal pull-up enabled in `Board_Ina219Init()`; external pull-up recommended | Board comments identify this as package pin 34 |
| INA219_I2C_SDA | PA10 | I2C1_SDA | INA219 SDA | Open-drain bus | Internal pull-up enabled in `Board_Ina219Init()`; external pull-up recommended | Board comments identify this as package pin 33 |
| INA219_VCC | 3.3V | N/A | INA219 VCC | N/A | N/A | Verify module IO voltage |
| INA219_GND | GND | N/A | INA219 GND | N/A | N/A | Common ground |
| INA219_IN+ | Sense input | N/A | Current sense high side | Analog | N/A | Board uses 5 mOhm shunt |
| INA219_IN- | Sense input | N/A | Current sense low side | Analog | N/A | Firmware currently inverts current sign |

INA219 configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Enabled | Yes | `FEATURE_ENABLE_INA219` |
| I2C instance | I2C1 | `INA219_I2C_INST` |
| Bus speed | 100 kHz | `INA219_I2C_BUS_SPEED_HZ` |
| 7-bit address | 0x40 | `BOARD_INA219_I2C_ADDRESS`; shell can scan 0x40 - 0x4F |
| Shunt resistor | 5 mOhm | `BOARD_INA219_SHUNT_MILLIOHMS` |
| Current LSB | 200 uA | `BOARD_INA219_CURRENT_LSB_UA` |
| Current sign | Inverted in firmware | `BOARD_INA219_INVERT_CURRENT = 1` |

## Status LED

| Signal | MCU Pin | Package Pin | External Connection | Direction | Active Level | Initial State | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| STATUS_LED | PB6 | 40 | Status LED circuit | MCU output | Low = on | Off | SysConfig initial value is SET |

## Buzzer

| Signal | MCU Pin | Package Pin | External Connection | Direction | Active Level | Initial State | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| BUZZER | PA12 | 51 | Active buzzer circuit | MCU output | High = on | Off | SysConfig initial value is CLEARED |

## Buttons

| Signal | MCU Pin | Package Pin | External Connection | Direction | Active Level | Pull-up / Pull-down | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| BUTTON1 | PC9 | 81 | Key to GND | MCU input | Low = pressed | Internal pull-up | 20 ms debounce |
| BUTTON2 | PB20 | 82 | Key to GND | MCU input | Low = pressed | Internal pull-up | 20 ms debounce |
| BUTTON3 | PB23 | 85 | Key to GND | MCU input | Low = pressed | Internal pull-up | 20 ms debounce |

## SWD Debug

| Signal | MCU Pin | External Connection | Direction | Notes |
| --- | --- | --- | --- | --- |
| SWDIO | PA19 | XDS110 / SWD debugger SWDIO | Bidirectional | Configured by Board module |
| SWCLK | PA20 | XDS110 / SWD debugger SWCLK | Debug input | Configured by Board module |
| GND | GND | Debugger GND | N/A | Common ground required |
| 3.3V sense | 3.3V | Debugger VTref, if used | N/A | Depends on debugger wiring |

## I2C Bus Summary

| Bus Alias | MCU Instance | SCL | SDA | Speed | Devices | Address Range Used |
| --- | --- | --- | --- | --- | --- | --- |
| `fram` | I2C2 | PC2 | PC3 | 400 kHz | FM24CL64B | Default 0x50 |
| `ina219` | I2C1 | PA11 | PA10 | 100 kHz | INA219 | Default 0x40, possible 0x40 - 0x4F |

## UART Summary

| Interface | MCU Instance | TX | RX | Baud | External Peer | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| DEBUG_UART | UART5 | PA1 | PA0 | 115200 | USB-UART / PC | Shell and logs |
| LORA_UART | UART0 | PB0 | PB1 | 115200 | LoRa module | Raw transparent serial |
| MOTOR_UART | UART1 | PA8 | PA9 | 115200 | MotorDriver | Binary motor control protocol |

## Unused / Reserved Interfaces

| Interface | Current State | Notes |
| --- | --- | --- |
| OLED | Disabled | `FEATURE_ENABLE_OLED = 0` |
| IMU | Disabled | `FEATURE_ENABLE_IMU = 0` |
| Local motor control | Disabled | `FEATURE_ENABLE_MOTOR = 0`; current motor control goes through MotorDriver UART |
| Encoder | Disabled | `FEATURE_ENABLE_ENCODER = 0` |
| ADC placeholder | No ADC driver registered | Shell command reports placeholder |
| PWM placeholder | No PWM driver registered | Shell command only echoes duty request |

## Bring-up Checklist

| Check | Expected Result | Shell Command |
| --- | --- | --- |
| Debug UART shell | Prompt responds at 115200 8N1 | `version` |
| Status LED | LED can turn on/off | `led on`, `led off` |
| Buzzer | Buzzer can turn on/off | `buzzer on`, `buzzer off` |
| Buttons | Pressed buttons read as pressed | `button` |
| FRAM bus idle | SCL/SDA high, read/write self-test passes | `fram status`, `fram test` |
| INA219 bus idle | SCL/SDA high, device address responds | `ina219 status`, `ina219 scan` |
| LoRa UART wiring | Module receives TX and gugaPI receives replies | `lora test`, `lora read` |
| MotorDriver UART wiring | Heartbeat returns OK | `motor ping`, `motor info` |

## Notes For Future Schematic Updates

| Item | Decision / Action |
| --- | --- |
| Do not edit generated files | Change `empty_cpp.syscfg`, then regenerate `Debug/ti_msp_dl_config.*` |
| Keep board pin ownership centralized | Add aliases in `board/board_pins.h` before wiring drivers |
| I2C pull-ups | Document exact resistor values from schematic when available |
| Connector names | Add J-number / pin-number mapping once schematic connector names are finalized |
| External power rails | Add rated voltage/current for LoRa, MotorDriver, INA219 sense path, and FRAM |
