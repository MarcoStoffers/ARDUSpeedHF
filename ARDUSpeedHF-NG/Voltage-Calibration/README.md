![ARDUSpeedHF-NG](https://marcostoffers.github.io/arduspeedhfNGlogo640.png)
## Arduino Sketch for Calibrating the LiPo Voltage
This Sketch will measure the battery voltage on the red and black wire and will output the ADC decimal value with 9600 Baud via the Receiver wire (orange cable). You can then connect the ESC with a TTL-UART to USB module to you computer and use any Terminalprogram to see the measured voltages (in ADC decimal value).

**Do not connect the +5V from the TTL UART to USB converter to the ESC !!! This will result in a interferience with the onboard 5V regulator !!!
The maximum voltage for the battery input is 15V (3S LiPo) !!!**

![XXDHW30A](https://marcostoffers.github.io/XXD-wiring.jpg)

## Requirement
- You need to install Arduino IDE version 1.8.12 or higher
- The hardware is not listed in the Arduino IDE. Therefore you need to install the "MiniCore" ( https://github.com/MCUdude/MiniCore ). There you can select the ATmega8 with internal 8MHz. Please see complete settings in the Sketch.
- You need to attach an AVRISP to the microcontroller (Microchip ATmega8). This can be done by soldering wires to it or to use special programming adapters (very rare).
- I'm using the Terminal program Termite ( https://www.compuphase.com/software_termite.htm ), which is very small and lightweight.

## Attention
The microcontroller ATmega8 is powered via an old 78L05 LDO. This type needs more than 6.7V (better 7V) to generate the 5V for the ATmega. So if you want to measure an empty 2S LiPo the system can stock. Better use a different LDO f.e. LE50CD-TR or LM2931AD-5.0R2G which are pin compatible. 

## Hardware Changes
It could be, that the TTL UART to USB converter will not recognize the serial signal from the ESC. This is due to a resistor, which is located directly to the signal-cable (orange). You can bridge this resistor with a small wire to fix this:
![Removed cable](https://marcostoffers.github.io/XXD-bridge-resistor.jpg)

## Measurement
* Connect the ESC to a variable Bench Powersupply
* Connect the ESC to the TTL UART to USB converter and plug in into your computer
* Start your Terminalprogram and switch on the Powersupply
* Adjust your voltage and note the ADC decimal value for this

LiPo voltages:
* 2S LiPo fully charged = 8.4V
* 2S LiPo slow down voltage = 6.6V (only 50% throttle)
* 2S LiPo cutoff voltage = 6.0V

* 3S LiPo fully charged = 12.6V
* 3S LiPo slow down voltage = 9.9V (only 50% throttle)
* 3S LiPo cutoff voltage = 9.0V

## LiFePo4 ???
Yes, you can also measure LiFePo4 voltages and change the values for 2S and 3S in the main code and your ESC is then ready for LiFePo4 use

LiFePo4 voltages:
* 2S LiFePo4 fully charged = 7.3V
* 2S LiFePo4 slow down voltage = 5.6V (only 50% throttle)
* 2S LiFePo4 cutoff voltage = 5.0V

* 3S LiFePo4 fully charged = 11.0V
* 3S LiFePo4 slow down voltage = 8.4V (only 50% throttle)
* 3S LiFePo4 cutoff voltage = 7.5V

 
## License
The project is licensed under the Creative Common License. A rebuild for private or non-profit associations is desired, commercial rebuild or distribution I forbid. If someone should develop this project further, I ask for the naming of the original project.

![CreativeCommonLicense](https://marcostoffers.github.io/cc.png)
