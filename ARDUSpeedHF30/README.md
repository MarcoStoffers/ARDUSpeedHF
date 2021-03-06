![ARDUSpeedHF30](https://marcostoffers.github.io/arduspeedhf30logo640.png)
## Arduino Sketch for the NoName 30A ESC for 3$
![Simonk30AESC](https://marcostoffers.github.io/simonk30a.png)

This sketch is for the 30A, red wrapped ESC you can find on eBay or Aliexpress for around 3$. This ESC is a clone of the HobbyPower 30A ESC.

## Requirement
- You need to attach an AVRISP to the microcontroller (Microchip ATmega8). This must be done via a programming clamp you can find here: https://www.thingiverse.com/thing:1569874
- The hardware is not listed in the Arduino IDE. Therefore you need to install the "MiniCore" ( https://github.com/MCUdude/MiniCore ). There you can select the ATmega8 with 8MHz internal

## Attention
The microcontroller ATmega8 is powered via an old 78L05 LDO. This type needs more than 6.7V (better 7V) to generate the 5V for the ATmega. So if you want to power the system with a 2S LiPo, be sure to use an external LiPo monitor, because the LDO will break down the voltage before the LiPo reaches the cut off voltage! You can change this by using a better LDO in SOT-89 housing (f.e. LD2981ABU50TR)

[EDIT] See new version 0.2 where I use the internal 2.56V voltage reference for the ADC. This is a little bit more stable and so a working LiPo cutoff for 2S.

## Setup
To setup the new programmed ESC to your RC you need to do the following steps:
* keep the throttle stick to full forward while powering the ESC - wait for a short Motorbeep
* pull the throttle stick to full reverse and wait again for a short Motorbeep
* bring the stick in the middle position and wait for 2 beeps
* decide if you want 100% reverse power or 50%. For 100% bring the stick to full forward and for 50% full reverse - wait for a beep
* bring the stick in the middle position and wait for 3 beeps
* decide if you want LiPo protect On or Off (2S -3S). For LiPo protect on bring the stick to full forward, for off to full reverse - wait for a beep
* bring the stick in the middle position. You will here 2 long Motorbeeps as a sign, that all values are stored in the internal EEPROM.

You can now use your new brushed ESC (no reboot required).

LiPo voltages:
* 2S LiPo fully charged = 8.4V
* 2S LiPo normal voltage = 7.4V
* 2S LiPo cutoff voltage = 6.6V

* 3S LiPo fully charged = 12.6V
* 3S LiPo normal voltage = 11.1V
* 3S LiPo cutoff voltage = 9.9V
 
## License
The project is licensed under the Creative Common License. A rebuild for private or non-profit associations is desired, commercial rebuild or distribution I forbid. If someone should develop this project further, I ask for the naming of the original project.

![CreativeCommonLicense](https://marcostoffers.github.io/cc.png)
