# JY-ME02-CAN Driver and Bench Test

> Historical test document. JY-ME02 is currently disabled in both feature
> profiles because the CAN bus is dedicated to one DM-G6220. Its source is
> retained for possible future multi-protocol dispatch, but its initialization
> and Shell commands are not registered in the current firmware.

## Scope

The first integration is intentionally read-only. Firmware receives and
decodes the encoder's measurement, temperature, and register-response frames.
The shell can issue the protocol's read-register command, but it does not
expose unlock, save, factory-reset, address-write, baud-write, zero, or
direction-write operations.

The application-layer receiver duplicates each hardware RX frame:

- one copy is passed to the JY-ME02 parser;
- one copy is retained for `can read` / `can watch`.

The cooperative task drains at most eight frames every 10 ms. No CAN or UART
operation blocks the main loop.

## Protocol Defaults

Factory defaults from the JY-ME02-CAN manual:

- standard CAN identifier: `0x050`;
- CAN bitrate: 250 kbit/s;
- output rate: 10 Hz;
- angular-velocity sample time: 1000 x 100 us = 100 ms;
- measurement payload: `55 55 angle_le16 speed_le16 revolutions_le16`;
- temperature payload: `55 56 temperature_le16 00 00 00 00`;
- read-register request: `FF AA 27 register 00`;
- register response: `55 5F value0_le16 value1_le16 value2_le16`.

The gugaPI CAN controller is configured for 250 kbit/s to match a factory
JY-ME02-CAN directly.

## Bitrate Commissioning

The initial commissioning setup keeps both gugaPI and a factory sensor at
250 kbit/s, so no persistent sensor configuration write is required.

1. Power off the bench.
2. Verify common ground and CANH/CANL polarity.
3. Measure CANH-to-CANL resistance with all power removed. Expect about 60 ohms
   only when exactly two 120-ohm end terminators are fitted.
4. Power the sensor from a suitable current-limited 5-36 V supply.
5. Configure USB-CAN for 250 kbit/s and confirm standard ID `0x050` frames whose
   first two bytes are `55 55` and/or `55 56`.
6. Connect gugaPI at its configured 250 kbit/s and confirm traffic before
   performing any motion test.

If the production bus is later migrated to 500 kbit/s, configure the sensor
with the vendor tool and retain a recovery path at 250 kbit/s. Do not use the
white ZERO wire during the communication test.

## Shell Commands

All numeric register addresses and CAN identifiers below are hexadecimal.

```text
can status
can clear
can watch on
jyme02 status
jyme02 readreg 04
jyme02 regs
jyme02 address 050
jyme02 sampletime 1000
```

`jyme02 address` and `jyme02 sampletime` change only the firmware parser. They
do not write the sensor. `sampletime` is decimal and uses 100-us units.

Register `0x04` is BAUD. A response starting at `0x04` with first value
`0x0002` means 500 kbit/s; `0x0004` means 250 kbit/s.

## Bench Regression

### 1. Passive receive

1. Run `can clear`, wait two seconds, then run `can status`.
2. Run `jyme02 status`.
3. Rotate the shaft slowly by hand in both directions.

Acceptance:

- CAN TEC/REC remain `0/0`, with no bus-off, warning, FIFO loss, or hardware RX
  drops;
- `measurement_count` increments at the configured output rate;
- `fresh(meas/temp)` reports `1/1` when both output frames are enabled;
- angle stays in approximately `0..359999` mdeg;
- revolution count changes only when crossing a full turn;
- velocity sign reverses when rotation direction reverses.

### 2. Scaling and wrap

1. Mark the shaft and housing.
2. Rotate exactly one turn clockwise over about two seconds.
3. Rotate exactly one turn counter-clockwise.

Acceptance:

- angle wraps once per mechanical revolution;
- the revolution counter changes by one and returns to its starting value;
- reported velocity magnitude is approximately 180 deg/s for each two-second
  turn, allowing for hand-motion error;
- no discontinuity is interpreted as a high-speed spike outside the actual
  wrap interval.

If the sensor sample-time register is changed, update the parser with
`jyme02 sampletime <value>` before assessing velocity scaling.

### 3. Register read

1. Run `jyme02 readreg 04`.
2. Wait at least 100 ms.
3. Run `jyme02 regs`.

Acceptance:

- the request reports `ok`;
- `register_count` increments;
- `regs start=0x04` is shown;
- the first returned value matches the active sensor bitrate setting.

### 4. Disconnect and recovery

1. Record a fresh `jyme02 status`.
2. Disconnect sensor CAN or power while leaving gugaPI powered.
3. Wait at least 500 ms and run `jyme02 status`.
4. Reconnect the sensor and repeat.

Acceptance:

- freshness changes to zero without blocking the scheduler or inhibiting the
  chassis;
- reconnecting restores fresh data without resetting gugaPI;
- CAN does not remain bus-off. If it does, capture `can status` before using
  `can recover`.

### 5. Bus-load soak

Run the sensor for ten minutes first at 10 Hz, then at 100 or 200 Hz if the
application requires it. Leave `can watch off` during the soak because serial
formatting is intentionally slower than CAN reception.

Acceptance:

- hardware `drop/fifo_lost` remain zero;
- TEC/REC remain zero;
- `measurement_count` continues increasing;
- scheduler and motor watchdog behavior remain normal.

The application diagnostic queue retains the latest 32 frames and may report
old-frame overwrites when it is not periodically read. This does not mean the
sensor parser dropped frames; use the hardware CAN counters and JY-ME02
measurement count to judge receive health.
