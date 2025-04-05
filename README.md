# RF Sensor Package 

## Introduction
### Description 
This application reads RF signal strength data from an RF sensor along with 
associated location data from a GPS at 5 seconds intervals. Between sensor
reads this data is transmitted to a receiver, where it is processed.
### Development Team
This project was created by students in the ECE Department at the FAMU-FSU 
College of Engineering as a senior design project in EEL4914C, taught by
Professor Oscar Chuy. 

## Hardware and Software
### Devices 
Sensors:
- Adafruit Ultimate GPS
- Mikroe RF-Meter
Microcontroller:
- STM32WL55JC1
### Toolchain
gcc-arm-none-eabi
### Debugging/Flashing
GDB/OpenOCD

## Project Structure

    src-----app             <-- application specific code such as classes for
         |                      sensor and configurations for used peripherals 
         |
         ---board           <-- contains peripheral settings, ISRs, and stubs
         |                      for standard library functions as needed
         |
         ---CMSIS           <-- board-specific library containing register
         |                      definitions, etc.
         |
         ---common          <-- contains general applications such as ring 
         |                      buffers and custom interrupt handling
         |
         ---drivers         <-- custom-made peripheral drivers (UART, GPIO, ec.)


