# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MultiStepMotor is a lightweight C library for managing multiple stepper motors on STM32 + FreeRTOS. It is designed to be integrated as a git submodule into STM32 projects. The library itself contains no build system — the host project's CMakeLists.txt compiles it via `add_subdirectory`.

## Architecture

Three files, all under the repo root:

- **stepper_motor.h** — Public API and type definitions (SM_HwConfig_t, enums for state/direction/stop-type)
- **stepper_motor.c** — All implementation: timer ISR pulse generation, FreeRTOS task for sleep management and completion callbacks, motor state machine
- **stepper_motor_config.h** — Default configuration with `#ifndef` guards so users can override via include-path priority

### Key Design Patterns

**Config override via include path**: Users create their own `stepper_motor_config.h` with `SM_COUNT`, `SM_Id_e`, and `sm_hw_table[]`. CMake places the user's Config/ directory before the library's include path, so the user's file wins. The library's copy provides only defaults for optional macros.

**Hardware abstraction via config table**: `SM_HwConfig_t` maps each motor ID to its GPIO pins, timer handle, and continuous-mode flag. No direct hardware references in the library code — everything goes through `sm_hw_table[]`.

**Timer ISR-driven pulse**: Two separate timer callbacks registered via `HAL_TIM_RegisterCallback` — `sm_timer_callback` for step-counted motors (counts toggles, 2 toggles = 1 step) and `sm_timer_continuous_callback` for continuous motors (just toggles CLK indefinitely).

**Completion via FreeRTOS queue + `__weak` callback**: ISR sends `SM_Report_t` to a queue; a FreeRTOS task polls it and calls `SM_ReportAction()` (weak, user overrides). Continuous motors do not send reports.

**Auto-sleep**: The task polls all motors every 10ms. If a motor has been idle for `SLEEP_TIMEOUT_MS` (default 3s), it disables the motor (SW pin high) and resets CLK.

### Speed Control

10 speed levels indexed 0–9 in `speed_curve_lut[]`. The LUT maps to timer auto-reload values in µs. Speed index is set per-motor via `SM_SetSpeed(id, speed)` where speed is 1–10 (converted to 0-based internally). Changing speed on a running motor takes effect on next `SM_Run()`.

## Conventions

- All public API functions are prefixed `SM_`
- All types use `_e` suffix for enums, `_t` suffix for structs
- Motor `id` is always validated against `SM_COUNT` at function entry
- Critical sections use `taskENTER_CRITICAL()`/`taskEXIT_CRITICAL()` (not ISR-safe variants) for task-level code; ISR code uses `xQueueSendFromISR` + `portYIELD_FROM_ISR`
- The `continuous` field in `SM_HwConfig_t` determines which timer callback is registered — step-counted vs continuous
- `SM_StopByLimit(id)` is called from external limit-switch logic, not internally
