/* -----------------------------------------------------------------------
** ArduSpeedHF-NG Voltage Calibration  
**
** Project: Speedcontroller for brushed Motors based on XXD-HW30A ESC from 
** banggood.com
** (https://github.com/MarcoStoffers/ARDUSpeedHF/tree/master/ARDUSpeedHF-NG)
**
** It measures the Input Voltage on the battery connection and outputs the
** decimal ADC value on the orange pin from the receiver wire with 9600 baud (8N1)
** 
** You will need to bridge the resistor near the orange wire from Receiver 
** connector!!!
**
** Based on the "MiniCore" https://github.com/MCUdude/MiniCore
** Schematic: https://github.com/NicksonYap/XXD-HW30A-ESC-Schematic
** 
** To program the ATmega8L you need the following setings in MiniCore:
** - ATmega8
** - internal 8MHz
** - B.O.D 2.7V
** - LTO disabled
** - No bootloader
** You will need to "burn bootloader" to set the right fuses to the ATmega8!!!
** 
** Revision History:
** 0.1: - initial Setup
** 0.2: - added Documentation
** =======================================================================
**
** Inputs:
** --------
** A0 - Battery Voltage Detection
**
** Outputs:
** ---------
** D2 - Serial Out
**
** Author: M.Stoffers
** Year: 2022
** License: CC (BY|NC|SA)
** -------------------------------------------------------------------------*/
#include <SoftwareSerial.h>

// -----------------------------
// User-defines
// -----------------------------

// -----------------------------
// System-defines
// -----------------------------
#define SERIAL_TX_PIN 2
#define SERIAL_RX_PIN 11
#define BATTERY_PIN A0

#define between(x, a, b)  (((a) <= (x)) && ((x) <= (b)))
// -----------------------------
// declares
// -----------------------------
SoftwareSerial mySerial(SERIAL_RX_PIN, SERIAL_TX_PIN);

// -----------------------------
// variables
// -----------------------------
uint16_t battery_voltage = 0;

// -----------------------------
// Setup
// -----------------------------
void setup() {
    analogReference(INTERNAL);                  // first set ADC Reference to internal 2.56V
    battery_voltage = analogRead(BATTERY_PIN);  // Init ADC

    mySerial.begin(9600);                       // Set the baud rate for the SoftwareSerial object

    mySerial.println("Read Battery Voltage and show it in decimal ADC value:");
    mySerial.println("Vmax = 15V !!!");
    mySerial.println();
}

// -----------------------------
// Mainprogram
// -----------------------------
void loop() {
       
    battery_voltage = analogRead(BATTERY_PIN);
    mySerial.println(battery_voltage, DEC);

    delay(500);
    
}
