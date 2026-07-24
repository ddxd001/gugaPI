# MotorDriver Hardware Interface Table

Fill this table from the schematic before editing SysConfig or firmware pin definitions.

## MCU

| Item | Value | Notes |
| --- | --- | --- |
| MCU part number | MSPM0G3519 | Update if the PCB uses a different device |
| Package | VQFN-48 | Update if different |
| Logic voltage | 3.3V | Example: 3.3 V |
| Motor supply voltage | 12V | Example: VM = 12 V / 24 V |
| Board revision | - | Not specified |

## I2C Slave Interface

| Signal | MCU Pin | External Connection | Pull-up | Notes |
| --- | --- | --- | --- | --- |
| I2C_SCL | PA17 | Main controller SCL | Internal pull-up | I2C1_SCL |
| I2C_SDA | PA18 | Main controller SDA | Internal pull-up | I2C1_SDA |
| I2C_IRQ / ALERT | - | Not connected | N/A | Optional |

I2C address rule:

| Field | Value |
| --- | --- |
| Base address | 0x20 |
| Address bits | DIP[2:0] |
| Address range | 0x20 - 0x27 |
| Final address formula | 0x20 + DIP[2:0] |

## I2C Address DIP Switches

| Signal | MCU Pin | Active Level | Pull-up / Pull-down | Address Bit | Notes |
| --- | --- | --- | --- | --- | --- |
| ADDR0 | PA4 | 0 | external 10k pull-up | bit 0 |  |
| ADDR1 | PA3 | 0 | external 10k pull-up | bit 1 |  |
| ADDR2 | PA2 | 0 | external 10k pull-up | bit 2 |  |

## UART Control Interface

Fill this section if the motor controller also needs command/control over a serial port.

| Signal | MCU Pin | MCU Peripheral | External Connection | Direction | Pull-up / Pull-down | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| UART_TX | PB17 | UART4_TX | Main controller RX / USB-UART RX | MCU output | N/A | 115200 8N1 |
| UART_RX | PB18 | UART4_RX | Main controller TX / USB-UART TX | MCU input | Internal pull-up | 115200 8N1 |
| UART_RTS | Not connected | N/A | Not connected | N/A | N/A | Hardware flow control not used |
| UART_CTS | Not connected | N/A | Not connected | N/A | N/A | Hardware flow control not used |
| UART_DE | Not connected | N/A | Not connected | N/A | N/A | RS-485 not used |
| UART_RE | Not connected | N/A | Not connected | N/A | N/A | RS-485 not used |
| UART_WAKE | Not connected | N/A | Not connected | N/A | N/A | Wake / interrupt line not used |

UART configuration:

| Field | Value | Notes |
| --- | --- | --- |
| Enabled | Yes | TX and RX only |
| Use case | Control | Control / debug log / both |
| Electrical interface | 3.3V TTL UART | Direct MCU-to-main-controller UART |
| Baud rate | 115200 | Recommended first bring-up value |
| Data bits | 8 | Recommended |
| Parity | None | Recommended |
| Stop bits | 1 | Recommended |
| Flow control | None | None / RTS-CTS / RS-485 DE |
| Protocol framing | Binary register frame | `SOF CMD REG LEN DATA... CRC8` |
| Shares commands with I2C | Yes | UART writes the same motor register map |

UART frame format:

| Byte Field | Size | Value / Meaning | Notes |
| --- | --- | --- | --- |
| SOF | 1 byte | 0xAA | Start of frame |
| CMD | 1 byte | 0x01 read, 0x02 write, 0x03 heartbeat | Command type |
| REG | 1 byte | 0x00 - 0xFF | Register address, same as I2C register map |
| LEN | 1 byte | 0 - 32 | Read length for CMD 0x01, write data length for CMD 0x02 |
| DATA | 0 - 32 bytes | Payload | Present only for write requests and data responses |
| CRC8 | 1 byte | CRC-8 poly 0x07, init 0x00 | Covers SOF through DATA |

UART response format:

| Request CMD | Response CMD | Response DATA |
| --- | --- | --- |
| 0x01 read | 0x81 | Register bytes |
| 0x02 write | 0x82 | 0x00 OK, 0x01 error |
| 0x03 heartbeat | 0x83 | 0x00 OK |

## DRV8701 Motor 1

| DRV8701 Pin / Signal | MCU Pin | MCU Peripheral | Direction | Active Level | Notes |
| --- | --- | --- | --- | --- | --- |
| IN1 / PH | PA5  | MCU output | TBD by test |  |
| IN2 / EN | PA6 | PWM | MCU output |  | TIMG0_CCP1 |
| nSLEEP | Not connected | Hardware fixed | N/A | Default enabled | MCU cannot put DRV8701 into sleep |
| nFAULT | Not connected | Hardware fixed | N/A | Low = fault | MCU cannot read DRV8701 fault directly |
| MODE | Not present | Fixed by DRV8701E variant | N/A | PH/EN interface | No MCU connection required |
| GAIN | Not used by firmware | Hardware fixed | N/A | N/A | No current sampling in firmware |
| VREF | Not controlled by firmware | Hardware fixed | N/A | N/A | Current limit reference, if hardware uses it |
| SO / current sense | Not connected / not used | Not used | N/A | N/A | No ADC current sampling |
| Other | - | N/A | N/A | N/A | No additional DRV8701 MCU signals |

## DRV8701 Motor 2

| DRV8701 Pin / Signal | MCU Pin | MCU Peripheral | Direction | Active Level | Notes |
| --- | --- | --- | --- | --- | --- |
| IN1 / PH | PB24 |  | MCU output | TBD by test |  |
| IN2 / EN | PA23 | PWM | MCU output |  | TIMG0_CCP0 (shared with M1) |
| nSLEEP | Not connected | Hardware fixed | N/A | Default enabled | MCU cannot put DRV8701 into sleep |
| nFAULT | Not connected | Hardware fixed | N/A | Low = fault | MCU cannot read DRV8701 fault directly |
| MODE | Not present | Fixed by DRV8701E variant | N/A | PH/EN interface | No MCU connection required |
| GAIN | Not used by firmware | Hardware fixed | N/A | N/A | No current sampling in firmware |
| VREF | Not controlled by firmware | Hardware fixed | N/A | N/A | Current limit reference, if hardware uses it |
| SO / current sense | Not connected / not used | Not used | N/A | N/A | No ADC current sampling |
| Other | - | N/A | N/A | N/A | No additional DRV8701 MCU signals |

## PWM Plan

| Motor | Timer Instance | PWM Channel | MCU Pin | Frequency | Duty Range | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| Motor 1 | TIMG0 | CCP1 | PA6 | 20 kHz target | 0 - 100% | DRV8701 EN input |
| Motor 2 | TIMG0 | CCP0 | PA23 | 20 kHz target | 0 - 100% | DRV8701 EN input, shared timer with M1 |

## ADC / Current Sense

| Signal | MCU Pin | ADC Instance | ADC Channel | Scale | Notes |
| --- | --- | --- | --- | --- | --- |
| M1 current sense | Not used | N/A | N/A | N/A | No ADC current sampling |
| M2 current sense | Not used | N/A | N/A | N/A | No ADC current sampling |
| VM sense | - | N/A | N/A | N/A | Not connected |
| Board temperature | - | N/A | N/A | N/A | Not connected |

## Fault And Protection Inputs

| Signal | MCU Pin | Source | Active Level | Firmware Action | Notes |
| --- | --- | --- | --- | --- | --- |
| M1_nFAULT | Not connected | DRV8701 Motor 1 | Low | Not available in firmware | Hardware-only fault indication, if any |
| M2_nFAULT | Not connected | DRV8701 Motor 2 | Low | Not available in firmware | Hardware-only fault indication, if any |
| E_STOP | - | Not connected | N/A | Not available in firmware | Optional |
| VM_GOOD | - | Not connected | N/A | Not available in firmware | Optional |

## Debug And Test

| Signal | MCU Pin | Function | Notes |
| --- | --- | --- | --- |
| SWDIO | PA19 | Debug | Existing template setting |
| SWCLK | PA20 | Debug | Existing template setting |
| UART_TX | - | Log output | Not connected |
| UART_RX | - | Log input | Not connected |
| STATUS_LED | - | Status output | Not connected |
| TEST_POINT_1 | - | Scope/debug | Not connected |

## Unused Pins Policy

| Policy Item | Decision | Notes |
| --- | --- | --- |
| Unused GPIO state | Input with internal pull-down | Default policy for unused pins |
| Boot-time motor state | PWM disabled | DRV8701 nSLEEP is hardware-enabled, so firmware must keep control outputs inactive at boot |
| Fault reset behavior | Not directly detectable from DRV nFAULT | Add current sense or other feedback if firmware fault handling is required |
| I2C timeout behavior | Coast | Disable EN PWM output on communication timeout |
| I2C timeout default | 1000 ms | Register 0x30 stores timeout in 10 ms units; default value 100 |
