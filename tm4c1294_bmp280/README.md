# EK-TM4C1294XL interface with Bosch BMP280
In this project, EK-TM4C1294XL board is interfaced with Bosch BMP280 Pressure and Temperature sensor using I2C.

## Requirements
* EK-TM4C1294XL - ARM® Cortex®-M4F-Based MCU TM4C1294 Connected LaunchPad™ Evaluation Kit
* Code Composer Studio (CCS) Integrated Development Environment (IDE)
* TivaWare™
* Realterm: Serial Terminal or any other terminal program software

## Peripheral Interface Used
* I2C Interface - This project uses EK-TM4C1294XLs I2C0 peripheral to connect with Bocsh BMP280
* UART Serial Interface - UART0 peripheral of the evaluation kit is used to communicate the Configuration, Pressure and Temperature data to the PC

## How it works
The microcontoller interfaces with the BMP280 and periodically sends the Pressure and Temperature data to PC.
The code is self explained with comments.

