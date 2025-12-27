# MotorControllerFirmware

[![CI](https://github.com/jvishnefske/MotorControllerFirmware/actions/workflows/ci.yml/badge.svg)](https://github.com/jvishnefske/MotorControllerFirmware/actions/workflows/ci.yml)

STM32F407-based motor controller firmware implementing Field Oriented Control (FOC) for driving up to 4 brushless motors simultaneously. Uses CMSIS-DSP for efficient Clarke/Park transforms and PID control loops.

## Quick Start

```bash
# Build firmware using Docker ARM toolchain
make build

# Clean build artifacts
make clean

# Flash to target (requires JLink)
JLinkGDBServer -if swd -device STM32F407ZE -nogui
```

## Architecture

- **Target MCU**: STM32F407ZETx (Cortex-M4, 168MHz, 512KB Flash, 192KB RAM)
- **FOC Algorithm**: Space Vector PWM with Clarke/Park transforms
- **Motor Channels**: 4 independent 3-phase PWM outputs (TIM1, TIM2, TIM3, TIM4)
- **Peripherals**: ADC (current sensing), CAN, USB CDC, I2C, SPI, UART

## Project Structure

```
Core/           - STM32CubeMX generated HAL initialization
signalProcessing/ - FOC algorithm implementation (cppmain.cpp)
Drivers/        - STM32 HAL and CMSIS drivers
startup/        - ARM Cortex-M4 startup assembly
```

## Debug

```bash
JLinkGDBServer -if swd -device STM32F407ZE -nogui -select ip=10.1.0.186
```
