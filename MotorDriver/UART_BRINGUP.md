# MotorDriver UART Bring-Up

Use this checklist before connecting motor power.

## 1. Empty Motor Power Test

Flash `Debug/MotorDriver.out`.

With motor power disconnected or driver outputs unloaded:

```text
motor ping
motor info
motor reg 0x00 6
```

Expected:

- `motor ping: ok`
- `id=0xA5`
- `fw=1`
- `i2c=0x00`

## 2. Register Write Test

```text
motor set 0x10 0x01 0x05 0x00
motor reg 0x10 4
motor set 0x10 0x00 0x00
motor set 0x20 0x01 0x05 0x00
motor reg 0x20 4
motor stop
```

Invalid writes should return error and leave registers unchanged:

```text
motor set 0x11 101
motor set 0x10 3
motor set 0x12 2
```

## 3. Scope Test

Keep motor power disconnected and probe PH/EN.

```text
motor m1 run 10 fwd
motor m1 run 50 rev
motor m1 coast
motor m2 run 10 fwd
motor m2 run 50 rev
motor stop
```

Expected:

- EN is 20 kHz PWM.
- Duty follows the requested percent.
- PH changes between forward and reverse.
- Coast and stop drive EN low.

## 4. Watchdog Test

```text
motor m1 run 10 fwd
```

Stop sending write commands. After about 1000 ms:

- M1 EN should go low.
- STATUS bit 3 should be set.
- FAULT_FLAGS bit 0 should be set.

Clear the fault:

```text
motor set 0x03 0x01
motor reg 0x02 2
```

## 5. Low Power Motor Test

Connect motor power after scope behavior is correct.

Start with low duty:

```text
motor m1 run 5 fwd
motor m1 run 10 fwd
motor m1 run 5 rev
motor m1 coast
```

Repeat for motor 2. Record actual mechanical direction before using higher duty.
