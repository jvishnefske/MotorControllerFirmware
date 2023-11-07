option(hardware_floating_point "use arm cortex hard float" ON)
message(STATUS "looking for toolchain")
set(ARM_CPU "cortex-m4")

#set(toolchain_paths /opt/gcc-arm-none-eabi-10.3-2021.10/bin)
set(toolchain_paths /opt/arm-gnu-toolchain-12.2.mpacbti-rel1-x86_64-arm-none-eabi/bin)
set(CMAKE_C_COMPILER ${toolchain_paths}/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${toolchain_paths}/arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER ${toolchain_paths}/arm-none-eabi-gcc)
set(CMAKE_AR ${toolchain_paths}/arm-none-eabi-ar)
set(CMAKE_OBJCOPY ${toolchain_paths}/arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP ${toolchain_paths}/arm-none-eabi-objdump)
set(SIZE a${toolchain_paths}/rm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_FIND_ROOT_PATH ${toolchain_paths})

set(ARCH_FLAGS -mcpu=cortex-m4)
set(CMAKE_CXX_FLAGS ${ARCH_FLAGS})
set(CMAKE_C_FLAGS ${ARCH_FLAGS})
set(CMAKE_ASM_FLAGS ${ARCH_FLAGS})
set(CMAKE_LDFLAGS_FLAGS ${ARCH_FLAGS})

set(ARM_CPU cortex-m4)
#include(signalProcessing/CMSIS_5/CMSIS/DSP/gcc.cmake)

#add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)

#if (hardware_floating_point)
#    # hardware floating point
#    add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#    add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#    add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#else ()
#    #software floating point
#    add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#    add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
#    add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
#endif ()