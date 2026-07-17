## Example Summary

MSPM0G3507 + DRV8323RS + AS5048B encoder-sensored FOC project using DriverLib.

The DRV8323RS is configured through SPI for 3x PWM mode. TIMA0 generates three
center-aligned 20 kHz waveforms on PA22/PA3/PA12. The AS5048B supplies the
14-bit rotor angle, a 20 kHz inner loop regulates d/q current, and a 100 Hz
speed loop regulates mechanical speed. An optional 100 Hz position loop drives
the speed loop using the shortest single-turn angle error. The commissioning
default is 25 RPM, and the motor has 14 pole pairs. It never
starts automatically after reset.

## DRV8323RS Connections

| MSPM0G3507 | DRV8323RS signal | Firmware use |
| --- | --- | --- |
| PA22 | INHA | TIMA0 C1, phase-A 20 kHz PWM |
| PA21 | CAL | Current-shunt amplifier calibration |
| PA18 | INLC | Phase-C half-bridge enable in 3x PWM mode |
| PA17 | nSCS | SPI chip select, controlled as GPIO |
| PA14 | SDI | SPI MOSI |
| PA13 | SDO | SPI MISO |
| PA12 | INHC | TIMA0 C3, phase-C 20 kHz PWM |
| PA11 | INLB | Phase-B half-bridge enable in 3x PWM mode |
| PA10 | ENABLE | Driver enable |
| PA6 | SCLK | SPI clock, 1 MHz, CPOL=0/CPHA=1 |
| PA5 | nFAULT | Active-low fault input and falling-edge interrupt |
| PA4 | INLA | Phase-A half-bridge enable in 3x PWM mode |
| PA3 | INHB | TIMA0 C2, phase-B 20 kHz PWM |
| PA8 | UART1 TX | Debug console, 115200 8N1 |
| PA9 | UART1 RX | Debug console, 115200 8N1 |
| PA24 | ADC0 channel 3 | DRV8323RS SOA current-sense output |
| PA25 | ADC0 channel 2 | DRV8323RS SOB current-sense output |
| PA15 | ADC1 channel 0 | DRV8323RS SOC current-sense output |
| PA0 | I2C0 SDA | AS5048B SDA, external 4.7 kOhm pull-up to 3.3 V |
| PA1 | I2C0 SCL | AS5048B SCL, external 4.7 kOhm pull-up to 3.3 V |

## Serial Commands

Open the PA8/PA9 serial port at 115200 baud, 8 data bits, no parity, and one
stop bit.

| Command | Action |
| --- | --- |
| `s` | Apply a small 500 ms alignment field, calibrate electrical zero, and start in speed mode |
| `p` | Start if needed, then capture the current shaft angle and enter position-hold mode |
| `v` | Leave position mode and select speed mode with a 0 RPM target |
| `x` | Stop and coast |
| `r` | Reverse the speed reference in speed mode |
| `+` / `-` | Speed mode: change magnitude by 25 RPM; position mode: move target by +/-10 degrees |
| `f` | Print FOC state, target/measured RPM, d/q currents, PWM duties, encoder count, and driver faults |
| `c` | Stop and clear latched faults |
| `i` | Sample SOA/SOB/SOC and print ADC counts, millivolts, zero-offset delta, and milliamps |
| `a` | Read AS5048B angle, magnitude, AGC, and diagnostic flags over I2C |
| `z` | Recalibrate the three current-sense zero offsets while the motor is stopped |
| `h` or `?` | Print command help |

The current-sense ADCs use VDDA (nominally 3.3 V) as their reference and four-sample
hardware averaging. The firmware explicitly configures the DRV8323RS CSAs for a
VREF/2 offset and 20 V/V gain, assumes 20 milliohm shunts, and calibrates zero-current
offsets at startup using the CAL pin. TIMG0 is phase-locked to TIMA0 and triggers
ADC0 and ADC1 together at 20 kHz during the all-low PWM interval, where the
low-side shunts are observable. The `i` command reports the latest synchronized
sample plus the signed average and absolute peak from the latest 256-sample
(12.8 ms) window.

The d-axis reference is zero. Each new alignment restores the conservative
25 RPM speed target. The speed PI includes output anti-windup and
releases stored torque when braking or reversing. It generates a q-axis current request
limited to about 250 mA. Three consecutive phase samples above about 600 mA
cause an immediate high-impedance protection stop. These are conservative
commissioning limits, not final motor-specific tuning values.

The custom board's measured SOA/SOB/SOC polarity is ADC count minus calibrated
VREF/2 offset for positive phase current. This differs from DRV8323RS EVM
layouts that define phase current as offset minus ADC count.

The AS5048B uses I2C0 at 400 kHz on PA0/PA1 and the default 7-bit address
0x40 (A1=A2=GND). A single repeated-start transaction reads registers 0xFA
through 0xFF. This provides AGC, diagnostics, magnetic magnitude, and the
14-bit absolute angle. While FOC is active, a shorter read of 0xFE/0xFF updates
the electrical angle continuously. At each start the rotor aligns to the alpha
axis and firmware derives the electrical offset from the measured mechanical
angle. Position mode limits its generated speed target to 50 RPM and switches
to a current-limited position/velocity controller inside 128 encoder counts
(about 2.81 degrees). An encoder update gap longer than 5 ms stops the
bridge in Hi-Z. Runtime angle reads use a 1 kHz non-blocking I2C state machine,
the current loop predicts electrical angle between samples, and UART TX uses a
non-blocking software queue serviced from the main loop.

## First-Power Safety

- Keep the DC-bus supply current-limited for initial testing.
- First test with the shaft unloaded and a low bus-current limit. Sending `s`
  intentionally pulls the rotor to a fixed position for 500 ms before rotation.
- Before applying the full bus voltage, confirm PA22/PA3/PA12 are synchronized
  20 kHz PWM waveforms and PA4/PA11/PA18 are high only while FOC is active.
- The selected gate-drive current and OCP values are conservative bring-up
  defaults; they must be adjusted for the actual MOSFET gate charge, shunt,
  bus voltage, motor, and PCB layout.
- DRV8323RS SDO and nFAULT are open-drain signals. Ensure the hardware has the
  required pull-ups; PA5 also enables the MCU's internal pull-up as a fallback.
- A low nFAULT immediately disables all three half bridges and leaves the motor
  coasting.

## Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| SYSCTL |  |  |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) for LaunchPad information, including user guide and hardware files.

| Pin | Peripheral | Function | LaunchPad Pin | LaunchPad Settings |
| --- | --- | --- | --- | --- |
| PA20 | DEBUGSS | SWCLK | N/A | <ul><li>PA20 is used by SWD during debugging<br><ul><li>`J101 15:16 ON` Connect to XDS-110 SWCLK while debugging<br><li>`J101 15:16 OFF` Disconnect from XDS-110 SWCLK if using pin in application</ul></ul> |
| PA19 | DEBUGSS | SWDIO | N/A | <ul><li>PA19 is used by SWD during debugging<br><ul><li>`J101 13:14 ON` Connect to XDS-110 SWDIO while debugging<br><li>`J101 13:14 OFF` Disconnect from XDS-110 SWDIO if using pin in application</ul></ul> |

### Device Migration Recommendations
This project was developed for a superset device included in the LP_MSPM0G3507 LaunchPad. Please
visit the [CCS User's Guide](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/tools/ccs_ide_guide/doc_guide/doc_guide-srcs/ccs_ide_guide.html#sysconfig-project-migration)
for information about migrating to other MSPM0 devices.

### Low-Power Recommendations
TI recommends to terminate unused pins by setting the corresponding functions to
GPIO and configure the pins to output low or input with internal
pullup/pulldown resistor.

SysConfig allows developers to easily configure unused pins by selecting **Board**→**Configure Unused Pins**.

For more information about jumper configuration to achieve low-power using the
MSPM0 LaunchPad, please visit the [LP-MSPM0G3507 User's Guide](https://www.ti.com/lit/slau873).

## Example Usage

Compile, load and run the example.
