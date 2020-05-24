# MKDrive Hardware Version 2

**+++Work in progress**

!MKDrive-2.0_top.png
!MKDrive-2.0_bottom.png

!MKDrive-2.0_render.png


## Components
* ATMEGA 328
* CAN: MCP2515 + TJA1050
* DRV8871

## Characteristics
* 6 .. 24 V
* 3.2 A
* CAN 2.0 (max. 1 Mbaud)
* 50 x 50 mm

## Changes to version 1
* Reverse polarity protection
* Fuse
* Measure battery current (INA213)
* Measure motor current (INA213)
* Added connector for serial 
* 4 Layers
* LEDs with different color (green = power, blue = programming, red = error)
* Removed reset button (NRTS is exposed on the AVRISP connector)

## Resources
* Board files: https://easyeda.com/mkaschub/mkdrive-v2
