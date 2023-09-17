# MotorControllerFirmware
[![.github/workflows/cmake.yml](https://github.com/jvishnefske/MotorControllerFirmware/actions/workflows/cmake.yml/badge.svg)](https://github.com/jvishnefske/MotorControllerFirmware/actions/workflows/cmake.yml)

This project was to demonstrait running Field Oriented Control algorithm for 4 motors on custom hardware. Currently the input to foc is simulated.  
It may be useful to connect foc input to to current adc, and desired torque at some point in order to close the control loop.

The non boilerplate part of the code is in the signalProcessing directory. The generated main() calls cppmain() with a structure of hardware handles.
