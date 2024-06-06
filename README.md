# Drivebot 

Learn how to make a robot drive around from scratch 

[Instructions](https://thefrontier.notion.site/Drivebot-Instructions-cc40ce03a2554190a8b564f60d7c01e6?pvs=74)

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