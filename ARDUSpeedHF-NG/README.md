![ARDUSpeedHF-NG](https://marcostoffers.github.io/arduspeedhfNGlogo640.png)
## Arduino Sketch for the XXD HW30A ESC
![XXDHW30A](https://marcostoffers.github.io/XXDHW30A.png)

This sketch is for the XXD HW30A ESC from Banggood.com ( https://www.banggood.com/Wholesale-XXD-HW30A-30A-Brushless-Motor-ESC-For-Airplane-Quadcopter-p-50621.html?rmmds=search&cur_warehouse=CN )
Due to the Hardware setup, I can use the build in Hardware-PWM to drive the Motor. This results in a 18kHz PWM frequency which is not hearable!

## Requirement
- You need to install Arduino IDE version 1.8.12 or higher
- The hardware is not listed in the Arduino IDE. Therefore you need to install the "MiniCore" ( https://github.com/MCUdude/MiniCore ). There you can select the ATmega8 with internal 8MHz. Please see complete settings in the Sketch.
- You need to attach an AVRISP to the microcontroller (Microchip ATmega8). This can be done by soldering wires to it or to use special programming adapters (very rare).

## Attention
The microcontroller ATmega8 is powered via an old 78L05 LDO. This type needs more than 6.7V (better 7V) to generate the 5V for the ATmega. So if you want to power the system with a 2S LiPo, better use a different LDO f.e. LE50CD-TR which is pin compatible. 

## Hardware Changes
To use the ESC with your brushed Motor (DC Motor) you need to remove the left Motor cable from the PCB to avoid shorts:
![Removed cable](https://marcostoffers.github.io/XXDHW30A-noleads.png)

To get a better cooling when a heatshrink is used, you can stick on a small RAM cooler with self-adhesife back:
![Add cooling](https://marcostoffers.github.io/XXDHW30A-cooling.png)

## Setup
To setup the new programmed ESC to your RC you need to do the following steps:
* keep the throttle stick to full forward while powering the ESC - wait for a short Motorbeep
* pull the throttle stick to full reverse and wait again for a short Motorbeep
* bring the stick in the middle position and wait for 2 beeps
* decide if you want 100% reverse power or 50%. For 100% bring the stick to full forward and for 50% full reverse - wait for a beep
* bring the stick in the middle position and wait for 3 beeps
* decide if you want LiPo protect On or Off (2S -3S). For LiPo protect on bring the stick to full forward, for off to full reverse - wait for a beep
* bring the stick in the middle position. You will here 2 long Motorbeeps as a sign, that all values are stored in the internal EEPROM.

You can now use your new brushed ESC (no reboot required). In LiPo Mode, the ESC will beep the number of LiPo cells during startup.

LiPo voltages:
* 2S LiPo fully charged = 8.4V
* 2S LiPo slow down voltage = 6.6V (only 50% throttle)
* 2S LiPo cutoff voltage = 6.0V

* 3S LiPo fully charged = 12.6V
* 3S LiPo slow down voltage = 9.9V (only 50% throttle)
* 3S LiPo cutoff voltage = 9.0V
 
## License
The project is licensed under the Creative Common License. A rebuild for private or non-profit associations is desired, commercial rebuild or distribution I forbid. If someone should develop this project further, I ask for the naming of the original project.

![CreativeCommonLicense](https://marcostoffers.github.io/cc.png)
