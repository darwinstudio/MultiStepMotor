# CODEBUDDY.md

This file provides guidance to CodeBuddy Code when working with code in this repository.

## Project Overview

MultiStepMotor is a lightweight C library for managing multiple stepper motors on STM32 + FreeRTOS. It is designed to be integrated as a git submodule into STM32 projects. The library itself contains **no build system** — the host project's `CMakeLists.txt` compiles it via `add_subdirectory`. Do not add a build system or try to build/test this repo standalone; there are no `build`, `lint`, or `test` commands here. All building and testing happens in the host (consumer) project.

## File Structure

Three files, all under the repo root:

- **stepper_motor.h** — Public API and type definitions (SM_HwConfig_t, enums for state/direction/stop-type)
- **stepper_motor.c** — All implementation: timer ISR pulse generation, FreeRTOS task for sleep management and completion callbacks, motor state machine
- **stepper_motor_config_template.h** — Configuration template; users copy it as `stepper_motor_config.h` in their own project, which takes priority via include-path order

## Architecture

### Config override via include path
The library ships `stepper_motor_config_template.h` as a starting point. Users copy it to their project as `stepper_motor_config.h` with `SM_COUNT`, `SM_Id_e`, and `sm_hw_table[]`. CMake places the user's `Config/` directory before the library's include path, so the user's file wins. The library's template only provides defaults for optional macros (`SM_DEFAULT_SPEED`, `SLEEP_TIMEOUT_MS`, `SM_TASK_STACK_SIZE`, `SM_TASK_PRIORITY`).

### Hardware abstraction via config table
`SM_HwConfig_t` maps each motor ID to its GPIO pins, timer handle, and continuous/no-sleep flags. No direct hardware references in the library code — everything goes through `sm_hw_table[]`. The table is `extern`-declared in the header and must be defined by the user in their own `.c` (e.g. `stepper_motor_config.c`).

### Timer ISR-driven pulse
Two separate timer callbacks registered via `HAL_TIM_RegisterCallback`:
- `sm_timer_callback` — for step-counted motors (counts toggles; 2 toggles = 1 step) and stops at target.
- `sm_timer_continuous_callback` — for continuous motors (just toggles CLK indefinitely).

The `continuous` field in `SM_HwConfig_t` selects which callback is registered per motor.

### Completion via FreeRTOS queue + `__weak` callback
The ISR sends `SM_Report_t` to a queue; a FreeRTOS task polls it and calls `SM_ReportAction()` (weak, user overrides). Continuous motors do not send reports.

### Auto-sleep
The task polls all motors every 10 ms. If a motor has been idle for `SLEEP_TIMEOUT_MS` (default 3 s), it disables the motor (SW pin high) and resets CLK. Motors with `no_sleep = 1` never auto-sleep (e.g. vertical axes that must hold current).

### Speed Control
10 speed levels indexed 0–9 in `sm_pulse_period_us[]`, mapping to timer auto-reload values in µs (50 µs ~ 450 µs period). Speed index is set per-motor via `SM_SetSpeed(id, speed)` where `speed` is 1–10 (converted to 0-based internally). Changing speed on a running motor takes effect on the next `SM_Run()`.

## STM32CubeMX Timer Requirements (critical)

Each motor timer must be configured with:
- **Auto-Reload Preload (ARPE) = Enable**
- **Period = 0**

CubeMX defaults Period to the max (65535 for 16-bit, 4294967295 for 32-bit). Because ARPE is on, ARR set via `HAL_TIM` only takes effect at the next update event. If Period stays at the default max, a 32-bit timer at 72 MHz needs ~59 s to reach the update event — the motor appears dead on startup. Setting Period = 0 makes `__HAL_TIM_SET_AUTORELOAD` take effect on the first count.

## Conventions

- All public API functions are prefixed `SM_`
- All types use `_e` suffix for enums, `_t` suffix for structs
- Motor `id` is always validated against `SM_COUNT` at function entry
- Critical sections use `taskENTER_CRITICAL()`/`taskEXIT_CRITICAL()` for task-level code; ISR code uses `xQueueSendFromISR` + `portYIELD_FROM_ISR`
- `SM_StopByLimit(id)` is called from external limit-switch logic, not internally

## Code Review Focus (this module is reused across multiple projects)

Be extra strict on correctness when editing — changes propagate to every consumer. Watch specifically for:

- **ISR vs task safety**: never call `taskENTER_CRITICAL()` from an ISR; never call `xQueueSend` (non-ISR) from timer callbacks. The two timer callbacks run in interrupt context.
- **No dynamic allocation**: the task/queue use FreeRTOS static allocation by design. Do not introduce `pvPortMalloc`/`new` or grow stack usage without adjusting `SM_TASK_STACK_SIZE`.
- **State machine integrity**: `SM_STATE_IDLE/READY/RUNNING` transitions, step-count accounting (2 toggles = 1 step), and the auto-sleep/idle-timer must stay consistent across `SM_Run`/`SM_StopContinuous`/`SM_StopByLimit`/`SM_Wake`/`SM_Sleep`.
- **Hardware pin safety**: SW (enable, active-low) state on stop/sleep, CLK reset to known state, DIR set before CLK starts. A wrong pin level can leave a motor energized or move it the wrong way.
- **`id` bounds** and NULL config/timer handles at every public entry.
- **Reentrancy of the LUT / globals**: `sm_vars[]` and `sm_pulse_period_us[]` are shared; protect cross-task access.

## Integration Summary (for host projects)

1. `git submodule add` this repo under e.g. `Middlewares/MultiStepMotor/`
2. Copy `stepper_motor_config_template.h` → `Config/stepper_motor_config.h`, define `SM_COUNT`, `SM_Id_e`, and `sm_hw_table[]`
3. In host `CMakeLists.txt`: `add_subdirectory(Middlewares/MultiStepMotor)` and put `Config/` before the library in `target_include_directories` so the user config wins
4. Call `SM_Init()`, then `SM_Run(id, dir, steps)` etc.

## Key API

`SM_Init`, `SM_Run`, `SM_StopContinuous`, `SM_GetState`, `SM_GetDir`, `SM_SetSpeed`, `SM_GetSpeed`, `SM_StopByLimit`, `SM_Wake`, `SM_Sleep`, `SM_ReportAction` (weak). See README.md for full signatures and usage examples.
