# MotorDriver I2C Protocol

The MCU is an I2C target device. The target address is selected by three active-low DIP switches.

## Address

| Item | Value |
| --- | --- |
| Base address | `0x20` |
| DIP bits | `ADDR2:ADDR0` |
| Active level | `0 = switch active` |
| Formula | `0x20 + DIP[2:0]` |
| Range | `0x20` to `0x27` |

## Register Access

Write one byte:

```text
START + SLA(W) + register + value + STOP
```

Read one or more bytes:

```text
START + SLA(W) + register + RESTART + SLA(R) + data... + STOP
```

Multi-byte writes and reads auto-increment the register address.

## Registers

| Address | Name | Access | Description |
| --- | --- | --- | --- |
| `0x00` | `DEVICE_ID` | R | Fixed `0xA5` |
| `0x01` | `FW_VERSION` | R | Firmware protocol version |
| `0x02` | `STATUS` | R | Global status flags |
| `0x03` | `FAULT_FLAGS` | R/W1C | Fault flags, write `1` to clear a bit |
| `0x04` | `CONTROL_FLAGS` | R/W | Global control flags |
| `0x05` | `I2C_ADDRESS` | R | Effective I2C address after DIP read |
| `0x10` | `M1_MODE` | R/W | Motor 1 mode |
| `0x11` | `M1_DUTY` | R/W | Motor 1 PWM duty, `0..100` |
| `0x12` | `M1_DIRECTION` | R/W | Motor 1 direction |
| `0x13` | `M1_STATUS` | R | Motor 1 status flags |
| `0x20` | `M2_MODE` | R/W | Motor 2 mode |
| `0x21` | `M2_DUTY` | R/W | Motor 2 PWM duty, `0..100` |
| `0x22` | `M2_DIRECTION` | R/W | Motor 2 direction |
| `0x23` | `M2_STATUS` | R | Motor 2 status flags |
| `0x30` | `WATCHDOG_TIMEOUT` | R/W | Timeout in 10 ms units, default `100` = 1000 ms |

## Mode Values

| Value | Name | Behavior |
| --- | --- | --- |
| `0` | `COAST` | PWM duty forced to 0 |
| `1` | `RUN` | PH set by direction, EN driven by PWM duty |
| `2` | `BRAKE` | Reserved as brake command; currently implemented as PWM off |

## Direction Values

| Value | Name | Notes |
| --- | --- | --- |
| `0` | `FORWARD` | Actual mechanical direction must be verified on hardware |
| `1` | `REVERSE` | Actual mechanical direction must be verified on hardware |

## Status Flags

`STATUS` register `0x02`:

| Bit | Mask | Meaning |
| --- | --- | --- |
| 0 | `0x01` | Firmware ready |
| 1 | `0x02` | Global enable set |
| 2 | `0x04` | At least one motor is in `RUN` mode |
| 3 | `0x08` | I2C watchdog timeout active |

`FAULT_FLAGS` register `0x03`:

| Bit | Mask | Meaning |
| --- | --- | --- |
| 0 | `0x01` | I2C watchdog timeout occurred |

`CONTROL_FLAGS` register `0x04`:

| Bit | Mask | Meaning |
| --- | --- | --- |
| 0 | `0x01` | Global motor enable |

`M1_STATUS` and `M2_STATUS`:

| Bit | Mask | Meaning |
| --- | --- | --- |
| 0 | `0x01` | Motor is in `RUN` mode |
| 1 | `0x02` | Direction register is `REVERSE` |

## Basic Bring-Up Test

1. Set DIP switches and scan for address `0x20..0x27`.
2. Read `DEVICE_ID` at `0x00`; expected value is `0xA5`.
3. Read `I2C_ADDRESS` at `0x05`; expected value is the active address.
4. Write `M1_DUTY = 5`, `M1_DIRECTION = 0`, `M1_MODE = 1`.
5. Verify Motor 1 output at low duty before increasing duty.
6. Write `M1_MODE = 0` to coast.
7. Repeat for Motor 2 registers at `0x20..0x22`.

