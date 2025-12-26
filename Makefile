# SPDX-License-Identifier: LGPL-2.1-or-later
# Motor Controller Firmware Makefile
# Builds STM32F407 firmware using Docker ARM toolchain

DOCKER_IMAGE := arm-none-eabi-gcc:13.2
BUILD_DIR := build
PROJECT := mcu2

.PHONY: all build clean docker-build

all: build

# Build using Docker with ARM toolchain
build:
	docker run --rm -v "$(CURDIR):/workspace" -w /workspace $(DOCKER_IMAGE) \
		/bin/bash -c "git submodule update --init --recursive && \
		cmake -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=Release && \
		cmake --build $(BUILD_DIR)"

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)

# Build Docker image with ARM toolchain (run once)
docker-build:
	docker build -t $(DOCKER_IMAGE) -f Dockerfile .

# Local build (requires arm-none-eabi-gcc installed)
local:
	git submodule update --init --recursive
	cmake -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=Release
	cmake --build $(BUILD_DIR)

# Flash firmware using JLink
flash: build
	JLinkExe -device STM32F407ZE -if SWD -speed 4000 -autoconnect 1 \
		-CommanderScript scripts/flash.jlink

# Start GDB server
gdb-server:
	JLinkGDBServer -if swd -device STM32F407ZE -nogui
