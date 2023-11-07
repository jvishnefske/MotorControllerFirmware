# build as top level project for testing, and developement
project(signalProcessing)
cmake_minimum_required(VERSION 3.10)
list(APPEND CMAKE_PREFIX_PATH $CMAKE_CURRENT_SOURCE_DIR/cmake)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/DSP/)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/DSP/Source)
# set cmsis root for dsp library
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5)
set(DSP ${ROOT}/CMSIS/DSP)
list(APPEND CMAKE_MODULE_PATH ${DSP})
set(ARM_CPU "cortex-m4")
#include(GCC) # cmsis toolchain file consider adding to top level build, or replacing.
add_subdirectory(${DSP}/Source bin_dsp)

add_library(signalProcessing
        src/cppmain.cpp
        src/KalmanFilter.cpp
        src/ChainableObserver.cpp
        )
target_include_directories(signalProcessing PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/public
        )
target_include_directories(signalProcessing PRIVATE
        ../
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_include_directories(signalProcessing PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5/CMSIS/DSP/Include)
target_link_libraries(signalProcessing PRIVATE
        CMSISDSPController
        CMSISDSPCommon
        )
target_compile_options(signalProcessing PRIVATE ${COMPILE_WARN_OPTIONS})
# skip unit test if we are compiling with arm-none-eabi
message(STATUS "Compiling with ${CMAKE_SYSTEM_PROCESSOR}")
# add test folder only if system processor is x86
if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86")
    message(STATUS "Adding test folder")
    add_subdirectory(test)
endif()

