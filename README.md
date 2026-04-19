# Vehicle Dynamics Code

This is an RTOS version of the original code *(+ some small additions)* running in Vehicle Dynamics board from 2022-2023. 

## Build Status

![Build Status](https://github.com/nikosstyl/Vehicle-Dynamics-2019-Code-RTOS/actions/workflows/build_release.yaml/badge.svg)

## QoL Improvements

* Compiling system changed from STM32CubeIDE to CMake using STM32CubeMX for initial code generation.
* Introduced CI/CD code compilation on each tag push. Each time a new tag is pushed using the following syntax `vx.x.x`, a new code compilation is being triggered. This effectively allows for only one place to efficiently distribute the firmware and eliminates duplicates.
* Added a YAML file to describe different board configurations. This is quite useful when multiple instances of the same board are used on the car but they don't use the same configs. A user can add as many board configurations as they like.
* Added a low pass filter on every ADC channel and on each IMU measurement with a configurable cut-off frequency. This frequency can be selected by sending a specific "command" to the board and it can be persistent or not, meaning it's being written to flash.

