# Drivebot 

Learn how to make a robot drive around from scratch 

## Prerequisites 
* Install PSoC Creator 
* A terminal emulator program, such as TeraTerm 

## 00_drivebot_mcuPSoC4_dev.cydsn
This project is for testing and developing firmware level code for Drivebot
* Uncomment the debug flags `MJL_DEBUG_<PROGRAM>` to test a specific piece of hardware

## 01_bootloader.cydsn 
The bootloader for the PSoC4200 µC

## 02_drivebot_app.cydsn
The main application code for drivebot. This is a bootloadable application. 

## drivebot
Common code used between the projects

## libmjl_driver_v1.0.184_cortex-m0
The embedded middleware library and pre-built binary 