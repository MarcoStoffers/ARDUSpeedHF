![ARDUSpeedHF](https://marcostoffers.github.io/arduspeedhf_logo_640.jpg)
## Arduino Sketch for the HobbyKing 20A & 30A UBEC ESC
![HobbyKing UBEC 20A](https://marcostoffers.github.io/hk20a.jpg)

This was the first Sketch to control a brushed Motor via a cheap brushless ESC from HobbyKing. This UBEC series of speed controller are available in different versions with different amps classified. This sketch is for the version 20A and 30A.

## Requirement
- You need to attach an AVRISP to the microcontroller (Microchip ATmega8). This can be done via the available pins or via a chip-adapter.
![Hobbyking UBEC 20A Pinout](https://marcostoffers.github.io/hk20apinout.jpg)

(Source: https://docs.google.com/spreadsheets/d/13tMlu5ldLNpZXwbe6UhDHJhcgTVuljm8HDiDp9WO9Pk/edit#gid=0) 
- The hardware is not listed in the Arduino IDE. Therefore you need to install the "MiniCore" ( https://github.com/MCUdude/MiniCore ). There you can select the ATmega8 with 16MHz crystal

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
The project is licensed under the Creative Common License. A rebuild for private or non-profit associations is desired, commercial rebuild or distribution I forbid. If someone should develop the mixer further, I ask for the naming of the original project.

![CreativeCommonLicense](https://marcostoffers.github.io/cc.png)
