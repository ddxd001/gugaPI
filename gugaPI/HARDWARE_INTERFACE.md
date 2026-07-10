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
PC2 / I2C2_SCL  -------->| SENSOR_I2C / IIC3 SCL|---- FM24CL64B SCL, INA219 SCL, OLED SCL
PC3 / I2C2_SDA  <------->| SENSOR_I2C / IIC3 SDA|---- FM24CL64B SDA, INA219 SDA, OLED SDA
                         |                      |
PA11 / I2C1_SCL -------->| MOTOR_I2C / IIC1 SCL |---- MotorDriver I2C SCL
PA10 / I2C1_SDA <------->| MOTOR_I2C / IIC1 SDA |---- MotorDriver I2C SDA
                         |                      |
PA29 / GPIO      -------->| GY931_SOFT_I2C SCL   |---- GY931 SCL
PA30 / GPIO      <------->| GY931_SOFT_I2C SDA   |---- GY931 SDA
                         |                      |
PB18 / SPIx_SCLK ------->| IMU_SPI SCLK         |---- ICM-45686 SCLK, LIS3MDLTR SCK
PB17 / SPIx_PICO ------->| IMU_SPI PICO/MOSI    |---- ICM-45686 SDI, LIS3MDLTR SDI
PB19 / SPIx_POCI <-------| IMU_SPI POCI/MISO    |---- ICM-45686 SDO, LIS3MDLTR SDO
PC7              --------| ICM45686_CS          |---- ICM-45686 CS
PC8              --------| LIS3MDLTR_CS         |---- LIS3MDLTR CS
                         |                      |
PB0 / UART0_TX  -------->| DEBUG_UART TX        |---- USB-UART RX / PC RX
PB1 / UART0_RX  <--------| DEBUG_UART RX        |---- USB-UART TX / PC TX
                         |                      |
PA8 / UART1_TX  -------->| MOTOR_UART TX        |---- MotorDriver RX
PA9 / UART1_RX  <--------| MOTOR_UART RX        |---- MotorDriver TX
                         |                      |
PA14 / UART3_TX -------->| LORA_UART TX         |---- LoRa RX
PA13 / UART3_RX <--------| LORA_UART RX         |---- LoRa TX
                         |                      |
PA27             --------| LED1                 |---- Active-low LED
PA26             --------| LED2                 |---- Active-low LED
PB27             --------| LED3                 |---- Active-low LED
PC16             --------| BUZZER               |---- Active-high buzzer
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
| External module power | LoRa, MotorDriver, INA219 sense side, FRAM, OLED, GY931 | Voltage and current depend on selected module |

## Debug UART

| Signal | MCU Pin | MCU Peripheral | External Connection | Direction | Pull-up / Pull-down | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| DEBUG_UART_TX | PB0 | UART0_TX | USB-UART RX / PC RX | MCU output | N/A | 115200 8N1 |
| DEBUG_UART_RX | PB1 | UART0_RX | USB-UART TX / PC TX | MCU input | Not configured in board code | 115200 8N1 |

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
| LORA_UART_TX | PA14 | UART3_TX | LoRa module RX | MCU output | N/A | Moved from PB0 so UART0 can host the shell |
| LORA_UART_RX | PA13 | UART3_RX | LoRa module TX | MCU input | Internal pull-up enabled in `Board_LoraInit()` | Moved from PB1 so UART0 can host the shell |
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
| Use case | Raw UART test and optional manual control | High-level shell control defaults to I2C; UART mode uses the binary MotorDriver frame |
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

## MotorDriver I2C Interface

This board uses a dedicated MotorDriver I2C bus. The board-level schematic name
is IIC1, and the MCU peripheral is I2C1.

| Signal | MCU Pin | MCU Peripheral | External Connection | Direction | Pull-up | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| MOTOR_I2C_SCL | PA11 | I2C1_SCL | MotorDriver SCL | Open-drain bus | External pull-up required by hardware; internal pull-up may be used during debug | Dedicated MotorDriver I2C bus |
| MOTOR_I2C_SDA | PA10 | I2C1_SDA | MotorDriver SDA | Open-drain bus | External pull-up required by hardware; internal pull-up may be used during debug | Dedicated MotorDriver I2C bus |
| MOTOR_GND | GND | N/A | MotorDriver GND | N/A | N/A | Common ground required |

MotorDriver I2C configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Enabled | Yes | `FEATURE_ENABLE_MOTOR_DRIVER` |
| Use case | Default MotorDriver high-level control | Shell `motor` commands default to this bus |
| Board bus name | IIC1 | Dedicated to MotorDriver |
| MCU I2C instance | I2C1 | `MOTOR_I2C_INST` |
| Bus speed | 100 kHz | Conservative bring-up speed |
| 7-bit address | 0x20 | `BOARD_MOTOR_DRIVER_I2C_ADDRESS` |
| Protocol | Direct register access | No UART frame wrapper on I2C |

## GY931 Angle Sensor I2C Interface

The WIT GY931 angle sensor is connected on PA29/PA30. These two pins can be
muxed to MSPM0 hardware I2C1/I2C2 functions, but both hardware controllers are
already assigned to MotorDriver and the shared SENSOR_I2C bus. Firmware
therefore uses GPIO bit-banged open-drain I2C for this module.

| Signal | MCU Pin | MCU Peripheral | External Connection | Direction | Pull-up | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| GY931_SOFT_I2C_SCL | PA29 | GPIO | GY931 SCL | Open-drain emulation | Internal pull-up enabled; external pull-up recommended | Dedicated software I2C clock |
| GY931_SOFT_I2C_SDA | PA30 | GPIO | GY931 SDA | Open-drain emulation | Internal pull-up enabled; external pull-up recommended | Dedicated software I2C data |
| GY931_VCC | 3.3V | N/A | GY931 VCC | N/A | N/A | Verify selected module voltage |
| GY931_GND | GND | N/A | GY931 GND | N/A | N/A | Common ground required |

GY931 configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Enabled | Yes | `FEATURE_ENABLE_GY931` |
| Board bus name | GY931 software I2C | Not registered in generic hardware I2C diagnostic table |
| MCU pins | PA29/SCL, PA30/SDA | `GPIO_GY931_I2C` SysConfig group |
| Bus speed | Conservative software I2C | `BOARD_GY931_I2C_HALF_PERIOD_CYCLES` controls timing |
| 7-bit address | 0x50 | `BOARD_GY931_I2C_ADDRESS`; shell can scan or change runtime address |
| Angle registers | Roll/Pitch/Yaw at 0x3D/0x3E/0x3F | WIT standard register map |
| Angle scale | raw / 32768 * 180 deg | Firmware reports fixed 0.001 deg units |

## FRAM I2C Interface

| Signal | MCU Pin | MCU Peripheral | External Connection | Direction | Pull-up | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| SENSOR_I2C_SCL | PC2 | I2C2_SCL | FM24CL64B SCL, INA219 SCL, OLED SCL | Open-drain bus | External pull-up required by hardware; firmware recovery uses GPIO mode | Board-level IIC3 shared bus; `PROJECT_GUIDE.md` identifies package pin 65 |
| SENSOR_I2C_SDA | PC3 | I2C2_SDA | FM24CL64B SDA, INA219 SDA, OLED SDA | Open-drain bus | External pull-up required by hardware; firmware recovery uses GPIO mode | Board-level IIC3 shared bus; `PROJECT_GUIDE.md` identifies package pin 66 |
| FRAM_VCC | 3.3V | N/A | FM24CL64B VCC | N/A | N/A | Verify with selected FRAM variant |
| FRAM_GND | GND | N/A | FM24CL64B GND | N/A | N/A | Common ground |

FRAM configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Enabled | Yes | `FEATURE_ENABLE_FRAM` |
| Device | FM24CL64B-GTR | 64 Kbit / 8 KiB I2C FRAM |
| Board bus name | IIC3 | Shared with INA219 and OLED |
| MCU I2C instance | I2C2 | `SENSOR_I2C_INST`; FRAM board alias uses this instance |
| Bus speed | 400 kHz | `SENSOR_I2C_BUS_SPEED_HZ` |
| 7-bit address | 0x50 | `BOARD_FRAM_I2C_ADDRESS` |
| Address pins | A2/A1/A0 hardware-dependent | Update address if strapped differently |
| Self-test address | 0x1FF0 | `BOARD_FRAM_SELF_TEST_ADDRESS` |

## OLED I2C Interface

The OLED module is Hansheng `HS91L02W2C01`, 0.91 inch, white, 128 x 32 dots.
Its datasheet lists `ADD:0x78`; that is the 8-bit write address, so firmware
uses 7-bit address `0x3C`.

| Signal | MCU Pin | MCU Peripheral | External Connection | Direction | Pull-up | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| SENSOR_I2C_SCL | PC2 | I2C2_SCL | OLED SCL, FM24CL64B SCL, INA219 SCL | Open-drain bus | External pull-up required by hardware; internal pull-up may be used during debug | Board-level IIC3 shared bus |
| SENSOR_I2C_SDA | PC3 | I2C2_SDA | OLED SDA, FM24CL64B SDA, INA219 SDA | Open-drain bus | External pull-up required by hardware; internal pull-up may be used during debug | Board-level IIC3 shared bus |
| OLED_VCC | 3.3V | N/A | OLED VCC | N/A | N/A | Datasheet supports logic up to 3.3 V |
| OLED_GND | GND | N/A | OLED GND | N/A | N/A | Common ground |

OLED configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Enabled | Yes | `FEATURE_ENABLE_OLED` |
| Device | HS91L02W2C01 | SSD1306-style I2C command set |
| Board bus name | IIC3 | Shared with FRAM and INA219 |
| MCU I2C instance | I2C2 | `SENSOR_I2C_INST`; OLED board alias uses this instance |
| Bus speed | 400 kHz | Shared bus speed |
| 7-bit address | 0x3C | `BOARD_OLED_I2C_ADDRESS`; datasheet `0x78` is the 8-bit write address |
| Resolution | 128 x 32 | `BOARD_OLED_WIDTH`, `BOARD_OLED_HEIGHT` |

## INA219 I2C Interface

| Signal | MCU Pin | MCU Peripheral | External Connection | Direction | Pull-up | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| SENSOR_I2C_SCL | PC2 | I2C2_SCL | INA219 SCL, FM24CL64B SCL, OLED SCL | Open-drain bus | Internal pull-up enabled in `Board_Ina219Init()` for debug; external pull-up recommended | Board-level IIC3 shared bus |
| SENSOR_I2C_SDA | PC3 | I2C2_SDA | INA219 SDA, FM24CL64B SDA, OLED SDA | Open-drain bus | Internal pull-up enabled in `Board_Ina219Init()` for debug; external pull-up recommended | Board-level IIC3 shared bus |
| INA219_VCC | 3.3V | N/A | INA219 VCC | N/A | N/A | Verify module IO voltage |
| INA219_GND | GND | N/A | INA219 GND | N/A | N/A | Common ground |
| INA219_IN+ | Sense input | N/A | Current sense high side | Analog | N/A | Board uses 5 mOhm shunt |
| INA219_IN- | Sense input | N/A | Current sense low side | Analog | N/A | Firmware currently inverts current sign |

INA219 configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Enabled | Yes | `FEATURE_ENABLE_INA219` |
| Board bus name | IIC3 | Shared with FRAM and OLED |
| MCU I2C instance | I2C2 | `SENSOR_I2C_INST`; INA219 board alias uses this instance |
| Bus speed | 400 kHz | Shared bus speed |
| 7-bit address | 0x40 | `BOARD_INA219_I2C_ADDRESS`; shell can scan 0x40 - 0x4F |
| Shunt resistor | 5 mOhm | `BOARD_INA219_SHUNT_MILLIOHMS` |
| Current LSB | 200 uA | `BOARD_INA219_CURRENT_LSB_UA` |
| Current sign | Inverted in firmware | `BOARD_INA219_INVERT_CURRENT = 1` |

## Grayscale Sensor (8-channel ADC)

An 8:1 analog multiplexer grayscale array. Three select GPIOs choose one of eight
phototransistor channels; the selected channel's analog voltage is read by a
single ADC input.

| Signal | MCU Pin | MCU Peripheral / GPIO | Direction | Notes |
| --- | --- | --- | --- | --- |
| SEL0 (bit0) | PC21 | GPIO_GRAY_C (OUTPUT) | MCU output | Channel address bit 0 |
| SEL1 (bit1) | PC20 | GPIO_GRAY_C (OUTPUT) | MCU output | Channel address bit 1 |
| SEL2 (bit2) | PA16 | GPIO_GRAY_A (OUTPUT) | MCU output | Channel address bit 2 (MSB) |
| AOUT | PA15 | ADC1 ADCIN0 (`GRAYSCALE_ADC`) | MCU input | Analog output of selected channel |

Grayscale configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Channels | 8 | channel = (PA16<<2) \| (PC20<<1) \| (PC21<<0) |
| ADC peripheral | ADC1 | PA15 is ADC1 ADCIN0 on MSPM0G3519 |
| Resolution | 12-bit | Result 0..4095, VREF = VDDA 3.3V |
| Sample time | 125 us | ULPCLK / 8, tunable in SysConfig if source impedance needs more |
| Settle time | ~100 us | `BOARD_GRAYSCALE_SETTLE_CYCLES`; after switching select before ADC sample |
| Select polarity | Active-high | Binary address driven directly on SEL0..SEL2 |
| SysConfig name | `GRAYSCALE_ADC`, `GPIO_GRAY_C`, `GPIO_GRAY_A` | |
| Board interface | `board/board_grayscale.h` | |
| Driver | `drivers/grayscale/` | |
| Feature switch | `FEATURE_ENABLE_GRAYSCALE` | |

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
| ICM-45686 INT2 MCU pin | PA31 | Use as INT2 interrupt input (open-drain output from sensor; sensor-side internal pull-up default on). FSYNC input mode not used. |
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
| Initial bus speed | 100 kHz | Conservative bring-up value; raise after data reads are stable |
| Max bus speed | 24 MHz | ICM-45686 datasheet limit; shared bus limited by slower device (LIS3MDL 10 MHz) |
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
| INT2 | TBD | PA31 | TBD | GPIO input | Sensor output (open-drain) | Active low (configurable) | Sensor-side internal pull-up default on; add external pull-up if sharing an open-drain bus | Same physical pin as FSYNC; used as INT2 interrupt input. MCU side is a no-pull input (sensor internal pull-up holds the line). Currently only polled in `imu status`; ISR not yet wired. |
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
| WHO_AM_I / device ID | 0xE9 @ reg 0x72 | Confirmed from ICM-45686 datasheet DS-000489 |
| SPI read/write command format | MSB=1 read, MSB=0 write; 7-bit addr; MSB-first; burst auto-increment; big-endian data | Confirmed from datasheet section 9.1 |
| Interrupt use | INT1 on PC6 for data-ready | INT polarity/function is register-configurable; verify from ICM-45686 datasheet during driver work |
| INT2 interrupt | PA31 used as INT2 interrupt input | Open-drain output from sensor; sensor internal pull-up default-enabled (regs `pads_int2_pe_trim_d2a[0]` / `pads_int2_pud_trim_d2a[0]`). MCU pin configured as INPUT (no pull). FSYNC input mode not used. |
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
| First firmware speed | 100 kHz | Increase only after WHO_AM_I and burst data reads are stable |
| Sensor orientation | Same orientation, +X forward | Confirm +Y/+Z direction before sensor fusion |

## LEDs

| Signal | MCU Pin | Package Pin | External Connection | Direction | Active Level | Initial State | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| LED1 | PA27 | 99 | LED1 circuit | MCU output | Low = on | Off | SysConfig initial value is SET |
| LED2 | PA26 | 98 | LED2 circuit | MCU output | Low = on | Off | SysConfig initial value is SET |
| LED3 | PB27 | 97 | LED3 circuit | MCU output | Low = on | Off | SysConfig initial value is SET |

## Buzzer

| Signal | MCU Pin | Package Pin | External Connection | Direction | Active Level | Initial State | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| BUZZER | PC16 | 35 | Active buzzer circuit | MCU output | High = on | Off | SysConfig initial value is CLEARED |

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

`IIC1` and `IIC3` are board-level bus names. On MSPM0G3519, the current
pinmux maps IIC1 to MCU `I2C1` and IIC3 to MCU `I2C2`.

| Bus Alias | MCU Instance | SCL | SDA | Speed | Devices | Address Range Used |
| --- | --- | --- | --- | --- | --- | --- |
| `motor` | I2C1 | PA11 | PA10 | 100 kHz | MotorDriver I2C target on dedicated IIC1 | Default 0x20, planned 0x20 - 0x27 |
| `gy931` | GPIO bit-bang | PA29 | PA30 | Software I2C | WIT GY931 angle sensor | Default 0x50 |
| `fram` | I2C2 | PC2 | PC3 | 400 kHz | FM24CL64B on shared IIC3 | Default 0x50 |
| `oled` | I2C2 | PC2 | PC3 | 400 kHz | HS91L02W2C01 OLED on shared IIC3 | Default 0x3C |
| `ina219` | I2C2 | PC2 | PC3 | 400 kHz | INA219 on shared IIC3 | Default 0x40, possible 0x40 - 0x4F |

## UART Summary

| Interface | MCU Instance | TX | RX | Baud | External Peer | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| DEBUG_UART | UART0 | PB0 | PB1 | 115200 | USB-UART / PC | Shell and logs |
| LORA_UART | UART3 | PA14 | PA13 | 115200 | LoRa module | Raw transparent serial |
| MOTOR_UART | UART1 | PA8 | PA9 | 115200 | MotorDriver | Binary motor control protocol |

## SPI Bus Summary

| Bus Alias | MCU Instance | SCLK | PICO / MOSI | POCI / MISO | Devices | Chip Selects | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `imu` | SPI0 | PB18 | PB17 | PB19 | ICM-45686, LIS3MDLTR | ICM45686_CS = PC7, LIS3MDLTR_CS = PC8 | Shared SPI bus; separate CS GPIOs |

## Unused / Reserved Interfaces

| Interface | Current State | Notes |
| --- | --- | --- |
| IMU / magnetometer | Planned | ICM-45686 and LIS3MDLTR shared SPI; `FEATURE_ENABLE_IMU = 0` until firmware is added |
| Local motor control | Disabled | `FEATURE_ENABLE_MOTOR = 0`; current motor control goes through external MotorDriver UART/I2C |
| Encoder | Disabled | `FEATURE_ENABLE_ENCODER = 0` |
| ADC placeholder | No ADC driver registered | Shell command reports placeholder |
| PWM placeholder | No PWM driver registered | Shell command only echoes duty request |

## Bring-up Checklist

| Check | Expected Result | Shell Command |
| --- | --- | --- |
| Debug UART shell | Prompt responds at 115200 8N1 | `version` |
| LEDs | LED1/LED2/LED3 can turn on/off/toggle | `led all status`, `led 1 on`, `led 2 on`, `led 3 on`, `led all off` |
| Buzzer | Buzzer can turn on/off | `buzzer on`, `buzzer off` |
| Buttons | Pressed buttons read as pressed | `button` |
| Shared IIC3 bus idle | PC2/PC3 high, FRAM read/write, INA219 probe, and OLED probe pass | `fram status`, `fram test`, `ina219 status`, `ina219 scan`, `oled status` |
| OLED display | Address 0x3C responds and test pattern is visible | `i2c scan oled 0x3C 0x3C`, `oled init`, `oled test` |
| GY931 software I2C | PA29/PA30 idle high, address 0x50 responds, angle values update when the module rotates | `gy931 status`, `gy931 scan 0x50 0x50`, `gy931 angle` |
| ICM-45686 + LIS3MDLTR SPI | Both WHO_AM_I reads return expected IDs | TBD after SPI driver and shell command are added |
| LoRa UART wiring | Module receives TX and gugaPI receives replies | `lora test`, `lora read` |
| MotorDriver UART wiring | Heartbeat returns OK in UART mode | `motor bus uart`, `motor ping`, `motor info` |
| MotorDriver I2C wiring | Address 0x20 responds on dedicated IIC1 motor bus | `i2c scan motor 0x20 0x27`, `motor bus i2c`, `motor info` |

## Notes For Future Schematic Updates

| Item | Decision / Action |
| --- | --- |
| Do not edit generated files | Change `empty_cpp.syscfg`, then regenerate `Debug/ti_msp_dl_config.*` |
| Keep board pin ownership centralized | Add aliases in `board/board_pins.h` before wiring drivers |
| I2C pull-ups | Document exact resistor values from schematic when available |
| Connector names | Add J-number / pin-number mapping once schematic connector names are finalized |
| External power rails | Add rated voltage/current for LoRa, MotorDriver, INA219 sense path, FRAM, OLED, and GY931 |
