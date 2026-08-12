# ODrive v0.5.1 source snapshot

This directory is a source snapshot copied from:

`D:\Odrive\程序源码\ODrive-fw-v0.5.1\ODrive-fw-v0.5.1`

The snapshot is kept for traceability while the product firmware is refactored
around `firmware/include` and `firmware/src`. It is not yet part of the
portable CMake test target because the original files depend on STM32 HAL,
FreeRTOS, generated Fibre interfaces, and board-wide globals.

## Retained for integration

- `MotorControl/motor.*`: FOC current loop, voltage modulation and calibration
- `MotorControl/encoder.*`: SPI AMS, ABZ, index and encoder PLL
- `MotorControl/sensorless_estimator.*`: nonlinear flux observer and PLL
- `MotorControl/controller.*`: torque, velocity and position loops
- `MotorControl/low_level.*`: ADC/PWM timing and disarm paths
- `Drivers/DRV8301`: gate-driver access
- `Board/v3`: M0 board peripheral configuration

Before these files join the product build, remove their Fibre endpoint base
classes and replace multi-axis/global dependencies with the product interfaces.
