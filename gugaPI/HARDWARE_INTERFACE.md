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
PB18 / SPIx_SCLK ------->| IMU_SPI SCLK         |---- ICM-45686 SCLK, LIS3MDLTR SCK
PB17 / SPIx_PICO ------->| IMU_SPI PICO/MOSI    |---- ICM-45686 SDI, LIS3MDLTR SDI
PB19 / SPIx_POCI <-------| IMU_SPI POCI/MISO    |---- ICM-45686 SDO, LIS3MDLTR SDO
PC7              --------| ICM45686_CS          |---- ICM-45686 CS
PC8              --------| LIS3MDLTR_CS         |---- LIS3MDLTR CS
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

## ICM-45686 + LIS3MDLTR Shared SPI Module

Use one MSPM0 SPI peripheral for both sensors. SCLK, PICO/MOSI, and POCI/MISO
are shared. Each IC must have its own chip-select GPIO, and any interrupt lines
should be routed to separate MCU GPIOs unless the schematic intentionally wires
them together.

### Schematic Fill-in Worksheet

Fill this worksheet first. The detailed tables below can then be updated from
the same information.

| Item | Current Value | Still Needed |
| --- | --- | --- |
| MSPM0 SPI instance | SPI0 | SysConfig routes PB17/PB18/PB19 to `IMU_SPI_INST = SPI0` |
| SPI SCLK MCU pin | PB18 | MCU package pin / schematic net name |
| SPI PICO / MOSI MCU pin | PB17 | MCU package pin / schematic net name |
| SPI POCI / MISO MCU pin | PB19 | MCU package pin / schematic net name |
| ICM-45686 CS MCU pin | PC7 | MCU package pin / schematic net name |
| LIS3MDLTR CS MCU pin | PC8 | MCU package pin / schematic net name |
| ICM-45686 INT1 MCU pin | PC6 | Use as data-ready interrupt |
| ICM-45686 INT2 / FSYNC MCU pin | PA31 | Use as FSYNC sync input / reserved; do not use as INT2 in first driver |
| LIS3MDLTR DRDY MCU pin | PA22 | Data-ready input |
| Sensor VDD net and voltage | 3.3V | Regulator net name still TBD |
| Sensor VDDIO net and voltage | 3.3V | Matches MSPM0 IO domain |
| CS pull-up resistors | 10k external pull-up to 3.3V | Applies to both CS lines |
| SPI series resistors | None | No series resistors on SCLK/MOSI/MISO |
| Sensor module connector | TBD | Connector name or schematic sheet reference |
| Board axis mapping | Same orientation, +X forward | Confirm +Y/+Z direction before sensor fusion |

### Shared SPI Bus

| Signal | Net Name | MCU Pin | MCU Package Pin | MCU Peripheral | ICM-45686 Pin / Signal | LIS3MDLTR Pin / Signal | Direction | Pull-up / Pull-down | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| IMU_SPI_SCLK | TBD | PB18 | TBD | SPI0_SCLK | SCLK | SCK | MCU output | None | Shared clock |
| IMU_SPI_PICO / MOSI | TBD | PB17 | TBD | SPI0_PICO / SPI0_MOSI | SDI / MOSI | SDI / MOSI | MCU output | None | Shared controller-to-device data |
| IMU_SPI_POCI / MISO | TBD | PB19 | TBD | SPI0_POCI / SPI0_MISO | SDO / MISO | SDO / MISO | MCU input | None | Shared device-to-controller data; verify tri-state when CS high |
| IMU_SPI_GND | GND | GND | N/A | N/A | GND | GND | N/A | N/A | Common ground required |

SPI bus configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Enabled | Planned | Enable `FEATURE_ENABLE_IMU` when firmware is added |
| MCU SPI instance | SPI0 | `IMU_SPI_INST` from SysConfig |
| Electrical interface | 3.3V CMOS | VDD and VDDIO are both 3.3V |
| SPI mode | Mode 3 target, CPOL = 1, CPHA = 1 | LIS3MDL requires clock idle high and captures on rising edge; verify ICM-45686 supports same mode |
| Bit order | MSB first | Required by LIS3MDL SPI protocol |
| Initial bus speed | 1 MHz | Conservative bring-up value |
| Max bus speed | 10 MHz until ICM limit is verified | LIS3MDL supports 10 MHz SPI; shared bus limited by slower device |
| Shared signals | SCLK, PICO/MOSI, POCI/MISO | Do not share CS |
| Separate signals | CS and INT pins | Assign per device below |
| SysConfig name | `IMU_SPI` | Suggested board-level instance name |
| Board wrapper | `board/board_imu.*` | Suggested future board abstraction |
| Driver folders | `drivers/icm45686/`, `drivers/lis3mdl/` | Suggested future driver layout |

### ICM-45686 Pin Table

| ICM-45686 Signal | Net Name | MCU Pin | MCU Package Pin | MCU Peripheral / GPIO | Direction | Active Level | Pull-up / Pull-down | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| SCLK | TBD | PB18 | TBD | IMU_SPI_SCLK | MCU output | N/A | None | Shared SPI clock |
| SDI / MOSI | TBD | PB17 | TBD | IMU_SPI_PICO / MOSI | MCU output | N/A | None | Shared with LIS3MDLTR |
| SDO / MISO | TBD | PB19 | TBD | IMU_SPI_POCI / MISO | MCU input | N/A | None | Shared with LIS3MDLTR |
| nCS / CSB | TBD | PC7 | TBD | GPIO output | MCU output | Low = selected | External 10k pull-up to 3.3V | Keep high during boot until SPI is ready |
| INT1 | TBD | PC6 | TBD | GPIO input / interrupt | Sensor output | Configurable / TBD | TBD | Configure as data-ready interrupt |
| INT2 / FSYNC | TBD | PA31 | TBD | GPIO input / timer sync TBD | Sync input / reserved | N/A for first driver | TBD | Same physical ICM-45686 pin; reserve for FSYNC, do not use as INT2 initially |
| AUX / other | Not connected | Not connected | N/A | Not connected | N/A | N/A | N/A | No other auxiliary pins connected |
| VDD | 3.3V | N/A | N/A | Power | N/A | N/A | N/A | Sensor core supply |
| VDDIO | 3.3V | N/A | N/A | Power | N/A | N/A | N/A | SPI IO supply |
| GND | GND | N/A | N/A | Ground | N/A | N/A | N/A | Common ground |

ICM-45686 configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Device | ICM-45686 | 6-axis IMU |
| Bus | Shared SPI | Same SPI peripheral as LIS3MDLTR |
| Chip select | PC7 | Dedicated GPIO, not shared |
| WHO_AM_I / device ID | TBD | Fill from datasheet or bring-up read |
| SPI read/write command format | TBD | Fill register address bit convention from datasheet |
| Interrupt use | INT1 on PC6 for data-ready | INT polarity/function is register-configurable; verify from ICM-45686 datasheet during driver work |
| FSYNC | PA31 reserved for sync input | Same physical pin as INT2; first driver should not use it as INT2 |
| Reset line | Not connected / TBD | Fill only if schematic has a reset pin connection |
| Axis orientation | Same as LIS3MDLTR, +X forward | Confirm +Y/+Z direction before sensor fusion |

### LIS3MDLTR Pin Table

| LIS3MDLTR Signal | Net Name | MCU Pin | MCU Package Pin | MCU Peripheral / GPIO | Direction | Active Level | Pull-up / Pull-down | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| SCK | TBD | PB18 | TBD | IMU_SPI_SCLK | MCU output | N/A | None | Shared SPI clock |
| SDI / MOSI | TBD | PB17 | TBD | IMU_SPI_PICO / MOSI | MCU output | N/A | None | Shared with ICM-45686 |
| SDO / MISO | TBD | PB19 | TBD | IMU_SPI_POCI / MISO | MCU input | N/A | None | Shared with ICM-45686 |
| CS | TBD | PC8 | TBD | GPIO output | MCU output | Low = selected | External 10k pull-up to 3.3V | Keep high during boot until SPI is ready |
| DRDY | TBD | PA22 | TBD | GPIO input / interrupt | Sensor output | Active high | TBD | LIS3MDLTR data-ready output |
| VDD | 3.3V | N/A | N/A | Power | N/A | N/A | N/A | Sensor supply |
| VDDIO | 3.3V | N/A | N/A | Power | N/A | N/A | N/A | SPI IO supply |
| GND | GND | N/A | N/A | Ground | N/A | N/A | N/A | Common ground |

LIS3MDLTR configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Device | LIS3MDLTR | 3-axis magnetometer |
| Bus | Shared SPI | Same SPI peripheral as ICM-45686 |
| Chip select | PC8 | Dedicated GPIO, not shared |
| WHO_AM_I / device ID | 0x3D at register 0x0F | From LIS3MDL datasheet |
| SPI read/write command format | bit7 = R/W, bit6 = auto-increment, bit5..0 = register address | Read sets bit7 = 1; multi-byte sets bit6 = 1 |
| Interrupt use | PA22 as DRDY | Data-ready output, active high |
| Reset line | Not connected | No reset line specified |
| Axis orientation | Same as ICM-45686, +X forward | Confirm +Y/+Z direction before sensor fusion |

### Sensor Power And Passive Components

| Item | Net / Value | Applies To | Notes |
| --- | --- | --- | --- |
| Sensor VDD | 3.3V | ICM-45686, LIS3MDLTR | Fill regulator net name if needed |
| Sensor VDDIO | 3.3V | ICM-45686, LIS3MDLTR | Matches MSPM0 SPI IO voltage |
| Local decoupling | TBD | Both sensors | Fill capacitor values from schematic |
| ICM-45686 CS pull-up | 10k to 3.3V | ICM-45686 | Keeps device deselected during reset |
| LIS3MDLTR CS pull-up | 10k to 3.3V | LIS3MDLTR | Keeps device deselected during reset |
| SPI SCLK series resistor | Not populated | Shared SPI | No series resistor |
| SPI MOSI series resistor | Not populated | Shared SPI | No series resistor |
| SPI MISO series resistor | Not populated | Shared SPI | No series resistor |
| INT pull resistors | TBD / None | Sensor interrupt lines | Fill if external resistors are used |

### Shared SPI Bring-up Rules

| Item | Decision / Action | Notes |
| --- | --- | --- |
| Boot CS state | Both CS high | Configure CS GPIOs high before enabling sensor transactions |
| Transaction rule | Assert exactly one CS at a time | Never select both sensors simultaneously |
| MISO contention check | Verify inactive device tri-states SDO/MISO | Scope MISO during first bring-up |
| First firmware speed | 1 MHz | Increase only after WHO_AM_I reads are stable |
| Sensor orientation | Same orientation, +X forward | Confirm +Y/+Z direction before sensor fusion |

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

## SPI Bus Summary

| Bus Alias | MCU Instance | SCLK | PICO / MOSI | POCI / MISO | Devices | Chip Selects | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `imu` | SPI0 | PB18 | PB17 | PB19 | ICM-45686, LIS3MDLTR | ICM45686_CS = PC7, LIS3MDLTR_CS = PC8 | Shared SPI bus; separate CS GPIOs |

## Unused / Reserved Interfaces

| Interface | Current State | Notes |
| --- | --- | --- |
| OLED | Disabled | `FEATURE_ENABLE_OLED = 0` |
| IMU / magnetometer | Planned | ICM-45686 and LIS3MDLTR shared SPI; `FEATURE_ENABLE_IMU = 0` until firmware is added |
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
| ICM-45686 + LIS3MDLTR SPI | Both WHO_AM_I reads return expected IDs | TBD after SPI driver and shell command are added |
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
