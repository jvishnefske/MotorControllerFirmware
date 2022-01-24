# build as top level project for testing, and developement
project(signalProcessing)
cmake_minimum_required(VERSION 3.10)
list(APPEND CMAKE_PREFIX_PATH $CMAKE_CURRENT_SOURCE_DIR/cmake)
add_subdirectory(CMSIS_5/CMSIS/DSP/Source)
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
# skip unit test if we are compiling with arm-none-eabi
message(STATUS "Compiling with ${CMAKE_SYSTEM_PROCESSOR}")
# add test folder only if system processor is x86
if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86")
    message(STATUS "Adding test folder")
    add_subdirectory(test)
endif()

