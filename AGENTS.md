# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## What this repository is

A TI Code Composer Studio (CCS) Theia workspace containing four independent embedded firmware projects targeting MSPM0 Arm Cortex-M0+ MCUs. There is no top-level build system; each project has its own `.cproject`/`.ccsproject` and is built by CCS managed make. Git remote is `github.com/ddxd001/gugaPI`; default branch `master`, active work on `feature/chassis-control`.

| Project | MCU | Language | Role |
| --- | --- | --- | --- |
| `FOC` | MSPM0G3507 | C | Sensorless→sensored BLDC FOC: DRV8323RS gate driver + AS5048B magnetic encoder. Standalone motor bench firmware. |
| `MotorDriver` | MSPM0G3519 | C++ | Dual DRV8701 PH/EN brushed motor driver. Exposes a UART/I2C register interface; is the *slave* to gugaPI. |
| `gugaPI` | MSPM0G3519 | C++ | Robot chassis controller. The *master*: runs the competition logic and commands MotorDriver over I2C/UART. |
| `TEST` | MSPM0G3519 | C | Minimal skeleton/scratch. Not part of the deployed system. |

The runtime system is **gugaPI ⇄ MotorDriver**: gugaPI issues speed/position commands and refreshes MotorDriver's watchdog every ~100 ms; on any comms failure gugaPI must stop both wheels. FOC is a separate single-motor bench project.

## Build, flash, debug

Toolchain: TI Arm Clang (`TICLANG_5.1.1.LTS`), `-O2 -Wall` — code must compile and link with **zero warnings**. SDK: MSPM0-SDK 2.10, SysConfig 1.26. Peripherals are configured in `.syscfg`; **never hand-edit the generated `Debug/ti_msp_dl_config.*`, `device.opt`, or `device_linker.cmd`** — change the `.syscfg` and let SysConfig regenerate.

CCS Theia regenerates `Debug/` makefiles from project sources. After adding a new `.cpp`/`.c` file, do a Clean Build in the IDE so it gets picked up.

Command-line build (this machine — gmake path reflects the CCS install location; the FOC handoff doc references an older `F:\CCS` path that does not exist here):

```bash
cd FOC/Debug        # or MotorDriver/Debug or gugaPI/Debug
C:/ti/ccs2100/ccs/utils/bin/gmake.exe -k -j 32 all -r -O
# outputs FOC.out / MotorDriver.out / gugaPI.out in that Debug/ dir
```

`Debug/` and `Release/` are gitignored build artifacts — do not edit source there. Flash via CCS using each project's `targetConfigs/*.ccxml` (launch configs in `.theia/launch.json`). Workspace-level `.theia/launch.json` contains machine-specific absolute paths — do not commit changes to it.

There is **no automated test suite**. "Testing" means bench regression: build, flash, and drive the hardware over the debug UART, following the procedures in `FOC/PROJECT_HANDOFF.md` §10, `MotorDriver/README.md`, and `gugaPI/docs/CHASSIS_CONTROL_PLAN.md`. Treat those three docs as authoritative for current parameter values and acceptance criteria.

Debug consoles (all 115200 8N1):
- FOC: UART1 PA8/PA9, single-char commands (`s` start, `x` stop, `f` status, `i` current ADC, `a` encoder, `n`/`t` CAN). See `FOC/README.md`.
- gugaPI: UART0 PB0/PB1, line-based shell. See `gugaPI/docs/SHELL_COMMANDS.md`.
- MotorDriver: UART4 PB17/PB18, `0xAA CMD REG LEN DATA... CRC8` frames.

## Architecture

### gugaPI layered architecture (enforced)

Per `gugaPI/docs/PROJECT_GUIDE.md`, dependency direction is fixed and one-way:

```
app  ->  services / algorithm / drivers  ->  board  ->  SysConfig / DriverLib
```

- **`app/`** — competition logic only (state machines, strategy, mode switching). Never touches registers directly.
- **`board/`** — pin/peripheral ownership and board init. Drivers must **not** hardcode pins; board passes resources to drivers via `Config` structs.
- **`drivers/<device>/`** — one dir per device (`xxx.h` + `xxx.cpp`). Init returns `DriverStatus` (from `drivers/common/driver_status.h`), never `void`. Severe faults escalate to `services::Fault_Set()`.
- **`services/`** — `time`, `scheduler`, `log`, `shell`, `debug_uart`, `fault`. App code calls these, never DriverLib directly.
- **`algorithm/`** — hardware-free math (PID, filters). Must not include `ti_msp_dl_config.h`.

Adding a device: `.syscfg` peripheral → `board/board_pins.h` registration → `drivers/<dev>/` → `board`/`app` calls `XXX_Init()` → register `XXX_Update()` with the scheduler if periodic.

### Cooperative scheduling, no RTOS

1 ms SysTick + cooperative scheduler (`services/scheduler`). `empty_cpp.cpp` `main()` runs `SYSCFG_DL_init() → Fault/Scheduler/Time/Board/Log/App_Init()`, then `while(1){ Scheduler_Run(); App_Run(); }`. Typical task periods: 1 ms buttons/encoders, 5 ms motor control, 10 ms sensors/IMU, 50 ms OLED, 100 ms telemetry.

**Tasks must not block or busy-wait on peripherals.** Slow buses need timeouts. ISRs do only: clear flag, read/write hardware FIFO, push to ring buffer, set a flag. No formatting, no long delays, no waiting on I2C/SPI/UART completion inside an ISR. Shell command parsing happens in `Shell_Process()` from the main loop, never in an ISR. UART RX is interrupt→ring-buffer; UART TX is non-blocking.

### gugaPI feature profiles

`config/feature_config.h` selects a build profile: `feature_development_config.h` (broad debug surface) vs `feature_competition_config.h` (only peripherals for the active robot path, suppressed chatter). Toggle `FEATURE_PROFILE_COMPETITION` to switch. Individual drivers are gated by `FEATURE_ENABLE_*` macros.

### FOC real-time control chain

`FOC/empty.c` is the monolithic main (UART/CAN/I2C/ADC/ISR entry); `FOC/motor_open_loop.c` holds the actual closed-loop FOC. **`motor_open_loop.*` is a legacy filename** — it now contains encoder-sensored FOC, not open-loop six-step. Do not rename it; the CCS managed-build file enumeration depends on the name. Control chain:

```
AS5048B mech angle -> 100Hz position loop (±50 RPM out) -> 100Hz speed PI (±250 mA Iq)
                  -> 20kHz d/q current PI -> inverse-Park + SVPWM -> DRV8323RS 3x 20kHz PWM
```

- **20 kHz current loop** runs in the ADC0 MEM1 ISR (interrupt priority 1, highest). TIMA0 center-aligned PWM; the three compare values load together at `ZERO_EVT` (counter zero) shadow event. TIMG0 is phase-locked to TIMA0 and triggers ADC0+ADC1 together during the all-low PWM window where low-side shunts are observable. The ADC0 ISR verifies ADC1's phase-C sample is ready before running the loop (see `j` command stats).
- **AS5048B angle** is read by a **1 kHz non-blocking I2C state machine** (timebase = 20 kHz ADC tick). The current loop *predicts* electrical angle between samples (max 2 ms extrapolation). Encoder gap > 5 ms → bridge goes Hi-Z.
- **UART TX** is a 1 KiB non-blocking queue serviced from the main loop (`u` shows queue/dropped). **The real-time path never waits on UART, CAN, or I2C.**
- **CAN**: 500 kbit/s classic CAN on PA26/PA27 via TCAN3413, PA23 STB held low (Normal). Receive filter accepts 0x000–0x7FF standard data frames into an 8-entry FIFO0; CAN ISR priority 2 (below ADC); FIFO drain + printing happen in main loop.
- All motion-control constants are centralized in `FOC/foc_config.h`. Change values there, not scattered in the modules, then re-run the bench regression in `DEVELOPMENT_PLAN.md`/`PROJECT_HANDOFF.md`.

## Critical, non-obvious invariants

- **FOC never auto-starts on reset.** Default state is STOP. Every `s` start does a 500 ms rotor alignment and recomputes the electrical-angle offset from the AS5048B reading. Currents/limits are conservative commissioning values, *not* production tuning.
- **FOC current polarity is `phase_current = ADC_count − calibrated_VREF/2_offset`** for this custom board. This is the opposite of DRV8323RS EVM layouts (`offset − ADC`). Do not "fix" it back.
- **Do not retarget the FOC ADC trigger** away from `TIMG0 → event channel 14 → ADC0/ADC1`. A previous attempt to publish directly from TIMA0 broke calibration (samples=0). Keep `MOTOR_FOC_enableAdcTrigger()`'s TIMG0↔TIMA0 phase sync.
- **MotorDriver physical mapping**: left wheel = M2, right wheel = M1; both 364 counts/rev on the output shaft. M1 counts via TIMG9 QEI, M2 via TIMG8 QEI (hardware quadrature, not GPIO interrupts).
- **MotorDriver watchdog**: active commands (`run duty>0`, `speed`, `speed 0`, `position`) refresh it; reads never do. UART heartbeat refreshes it; I2C `motor ping` does *not* (it only probes the address). Watchdog default 1 s; timeout forces both channels coast + sets `FAULT_FLAGS bit0`.
- **GY931 on gugaPI uses software/bitbang I2C on PA29/PA30** deliberately, to avoid stealing hardware I2C1 (MotorDriver) or I2C2 (FRAM/INA219/OLED). Don't move it to a hardware I2C instance without checking bus conflicts.
- **Default gugaPI→MotorDriver bus is I2C** (addr 0x20). `motor bus uart` switches the high-level command path; raw `motor send/hex/read` always use UART.

## Development guidance

- Match surrounding code's style and comment density. FOC is C with heavy inline comments; gugaPI/MotorDriver are C++ with the `XXX_Init/Update/Read` naming convention and `Config`/`Context` structs.
- Each completed build/flash/bench-regression stage commits to `feature/chassis-control` with key serial logs and hardware version noted (per `FOC/PROJECT_HANDOFF.md` §13).
- Hardware is live and current-limited during bring-up. When touching motor/control code, prefer the conservative defaults and the documented bench procedure over aggressive retuning. On oscillation, overcurrent, or angle divergence → `x` stop and capture the full serial log before changing parameters.
