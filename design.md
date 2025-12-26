# Motor Controller Firmware Design

## MVP Functional Requirements

### Core Motor Control
- [ ] FR-001: Initialize STM32F407 system clock to 168MHz from 25MHz HSE
- [ ] FR-002: Configure 4 independent PWM timer channels (TIM1, TIM2, TIM3, TIM4) for 3-phase output
- [ ] FR-003: Implement Clarke transform (abc to alpha-beta)
- [ ] FR-004: Implement Park transform (alpha-beta to d-q)
- [ ] FR-005: Implement inverse Park transform (d-q to alpha-beta)
- [ ] FR-006: Implement inverse Clarke transform (alpha-beta to abc)
- [ ] FR-007: Generate center-aligned PWM at configurable frequency (default 40kHz)
- [ ] FR-008: Implement PID controller for theta (rotor angle) tracking

### Safety
- [ ] FR-009: Configure independent watchdog (IWDG) with timeout
- [ ] FR-010: Implement Error_Handler for fault conditions
- [ ] FR-011: Provide PWM output enable/disable control per channel

### Peripheral Interfaces
- [ ] FR-012: Initialize ADC channels for current measurement (ADC1, ADC2, ADC3)
- [ ] FR-013: Configure CAN bus interface for external commands
- [ ] FR-014: Initialize USB CDC for debug/configuration interface
- [ ] FR-015: Configure UART interfaces (USART1-3, USART6)
- [ ] FR-016: Initialize I2C interfaces (I2C1, I2C2)
- [ ] FR-017: Configure SPI interface (SPI3) for LCD display

### Build System
- [x] FR-018: Cross-compile for ARM Cortex-M4 using arm-none-eabi-gcc
- [x] FR-019: Generate .elf, .hex, and .bin output files
- [x] FR-020: Support Docker-based CI build environment

## Non-Functional Requirements

- NFR-001: PWM update rate shall support motor control loop at 40kHz minimum
- NFR-002: Code shall compile with -Werror=all -Werror=extra -Werror=pedantic
- NFR-003: Firmware binary shall fit within 512KB Flash
- NFR-004: RAM usage shall not exceed 192KB

## Traceability

| Requirement | Implementation | Test |
|-------------|----------------|------|
| FR-001 | Core/Src/main.c:SystemClock_Config() | CI build |
| FR-002 | Core/Src/main.c:MX_TIM*_Init() | CI build |
| FR-003 | signalProcessing/src/cppmain.cpp:arm_clarke_f32() | CI build |
| FR-004 | signalProcessing/src/cppmain.cpp:arm_park_f32() | CI build |
| FR-005 | signalProcessing/src/cppmain.cpp:arm_inv_park_f32() | CI build |
| FR-006 | signalProcessing/src/cppmain.cpp:arm_inv_clarke_f32() | CI build |
| FR-007 | signalProcessing/src/cppmain.cpp:ThreePhasePwm | CI build |
| FR-008 | signalProcessing/src/cppmain.cpp:arm_pid_f32() | CI build |
| FR-009 | Core/Src/main.c:MX_IWDG_Init() | CI build |
| FR-010 | Core/Src/main.c:Error_Handler() | CI build |
| FR-011 | signalProcessing/src/cppmain.cpp:enable_output()/disable_output() | CI build |
| FR-018 | Makefile, CMakeLists.txt | CI workflow |
| FR-019 | CMakeLists.txt:add_custom_command() | CI artifact |
| FR-020 | .github/workflows/ci.yml | CI pass |
