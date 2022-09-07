/* -----------------------------------------------------------------------
** ArduSpeedHF NG (Next Generation) V0.4 - Speedcontroller for brushed  
** Motors based on XXD-HW30A ESC at banggood.com
** Schematic: https://raw.githubusercontent.com/NicksonYap/XXD-HW30A-ESC-Schematic/master/schematic.png
**
** HSb           HSc
**  o             o
**  |             |
**  |             |
**  o--- Motor ---o
**  |             |
**  |             |
**  o             o
** LSb           LSc
** OC1A          OC1B
**
** based on the "MiniCore" https://github.com/MCUdude/MiniCore
** ATmega8L PINout: https://camo.githubusercontent.com/1158321594ca3f9ba348093edabc9fb3791a311c/68747470733a2f2f692e696d6775722e636f6d2f6e6177657145362e6a7067
** To program the ATmega8L you need the following setings in MiniCore:
** - ATmega8
** - internal 8MHz
** - B.O.D 2.7V
** - LTO disabled
** - No bootloader
** You will need to "burn bootloader" to set the right fuses to the ATmega8!!!
** 
** Revision History:
** 0.1: - initial Setup - not working (will cause shorts between motor MosFETs
** 0.2: - change Timer1 inverse OCR1A & OCR1B and correct code 
**      - change Timer1 PWM frequency to 18kHz
**      - change Timer1 PWM frequency for motor beeping to 1kHz
**      - first real Test
** 0.3: - change LiPo check values (35.5 per V)
**      - change LiPo limit to 3.3V per cell
**      - change LiPo cut off to 3.0V per cell
** 0.4: - add documentation
** 0.5: - add User defines for RCmin and RCmax
** =======================================================================
**
** Startup: 
** --------
** - the ESC will learn RC middle in the first second after switch on!
** - if LiPo check is enabled, the motor will beep the numbers of LiPo cells (2 or 3)
** 
** Setup:
** ------
** 1) Stick in max up-position -> wait for motor beep
** 2) Stick in max down-position -> wait for motor beep
** 3) Stick in center position -> wait for 2 motor beeps
** 4) Stick in max up-position for 100% or down-position for 50% reverse power -> wait for motor beep
** 5) Stick in center position -> wait for 3 motor beeps
** 6) Stick in max up-position for LiPo protect on or down-position for off -> wait for motor beep
** 7) Stick in center position -> wait for 2 long motor beeps
** 8) Done ! No restart required
**
** Inputs:
** --------
** D2 - RC input
** A0 - Battery Voltage Detection
**
** Outputs: you need to identify the OC1A and OC1B ports. These ports are for PWM output. The opposite site of MosFETs
** (high side) must then correctly set to pFET A and pFET B!
** ---------
** D3 - HSa (not used - only for documentation - will be set to low)
** D4 - HSb
** D5 - HSc
** D8 - LSa (not used - only for documentation - will be set to low)
** D9 - LSb (OC1A)
** D10- LSc (OC1B)  
**
** EEPROM data:
** -------------
** 1 - max RC value
** 3 - min RC value
** 5 - 50% reverse (1=true / 0=false)
** 6 - lipo check (1=on / 0=off)
**
** LiPo analog read values:
** ------------------------
** 2S low (6,0V) = 213
** 2S mid (6,6V) = 235
** 2S high (8,4V) = 298
** 3S low (9,0V) = 320
** 3S mid (9,9V) = 352
** 3S high (12,6V) = 448
**
** Author: M.Stoffers
** Year: 2020
** License: CC (BY|NC|SA)
** -------------------------------------------------------------------------*/
#include <avr/eeprom.h>

// -----------------------------
// User-defines
// -----------------------------
#define Hysterese_for 50        // Hysterese for stick from middle position to motor starts forward (deadzone of stick)
#define Hysterese_rev 50        // Hysterese for stick from middle position to motor starts reverse (deadzone of stick)

#define RCmin 900               // Value for RCmin - please change accordingly to your needs
#define RCmax 2100              // Value for RCmax - please change accordingly to your needs

// -----------------------------
// System-defines
// -----------------------------
#define HSa 3
#define HSb 4
#define HSc 5
#define LSa 8
#define LSb 9
#define LSc 10

#define RC_PIN 2
#define BATTERY_PIN A0

#define lipo_2s_low 213
#define lipo_2s_mid 235
#define lipo_2s_high 298
#define lipo_3s_low 320
#define lipo_3s_mid 352
#define lipo_3s_high 448

#define between(x, a, b)  (((a) <= (x)) && ((x) <= (b)))
// -----------------------------
// declares
// -----------------------------

// -----------------------------
// variables
// -----------------------------
uint8_t helper;
uint8_t pin_array[6] {HSa, HSb, HSc, LSa, LSb, LSc};

volatile unsigned long start_micros;
volatile unsigned long stop_micros;

volatile uint16_t rc_temp;
volatile uint16_t rc_value;
volatile uint8_t rc_avg_count;
volatile boolean rc_ok = false;
uint16_t rc_middle = 0;
uint16_t rc_max = 0;
uint16_t rc_min = 0;
uint16_t rc_value_last;

uint16_t pwm_full, pwm_limit, pwm_motor_beep, pwm_motor_beep_on;
boolean mot_ok = false, reverse_half = false;

uint16_t battery_voltage = 0;
uint8_t lipo_cells = 0;
boolean lipo_ok = false, lipo_limit = false, lipo_check = false;

// -----------------------------
// Setup
// -----------------------------
void setup() {
    analogReference(INTERNAL);                                                                    // first set ADC Reference to internal 2.56V
    
    for(helper = 0; helper < 6; helper++) {                                                       // set all MosFET Pins ...
      pinMode(pin_array[helper], OUTPUT);                                                         // ... as Output ...
      digitalWrite(pin_array[helper], LOW);                                                       // ...  and to LOW
    }
       
    attachInterrupt(digitalPinToInterrupt(RC_PIN), RC_Interrupt, CHANGE);                         // start interrupt for RC meassuring
    delay(500);                                                                                   // ... and wait for the signal to become stable

    if(F_CPU == 16000000 ) {
      pwm_full = 890;                                                                             // Set pwm_full ...
      pwm_limit = 445;                                                                            // and pwm_limit for 16MHz (18kHz PWM)
      pwm_motor_beep = 20000;                                                                     // Set PWM for Motor beep (850Hz)
      pwm_motor_beep_on = 19000;                                                                  // and PWM Duty-Cycle to 5%
    }
    if(F_CPU == 8000000 ) {
      pwm_full = 445;                                                                             // Set pwm_full ...
      pwm_limit = 222;                                                                            // and pwm_limit for 8MHz (8kHz PWM)
      pwm_motor_beep = 10000;                                                                     // Set PWM for Motor beep (850Hz)
      pwm_motor_beep_on = 9500;                                                                   // and PWM Duty-Cycle to 5%
    }

    init_timer1();
            
    learn_middle();                                                                               // learn stick middle and setup min & max if wanted
    
    rc_max = eeprom_read_word((uint16_t*)1);                                                      // read EEPROM value for RC max
    if(!between(rc_max, RCmin, RCmax)) rc_max = 2000;                                             // and check if valid
    
    rc_min = eeprom_read_word((uint16_t*)3);                                                      // read EEPROM value for RC min
    if(!between(rc_min, RCmin, RCmax)) rc_min = 1000;                                             // and check if valid

    if(eeprom_read_byte((uint8_t*)5) == 1) reverse_half = true;                                   // check if reverse limit is set and set reverse value to 50%
    
    if(eeprom_read_byte((uint8_t*)6) == 1) lipo_check = true;                                     // check if lipo check is set to on
    else lipo_check = false;                                                                      // else switch off
        
    if(lipo_check) {                                                                              // if LiPo check is enabled ...
        battery_voltage = analogRead(BATTERY_PIN);
        
        if(between(battery_voltage,lipo_2s_low,lipo_2s_high)) {                                   // ... check if it is a 2S battery
            lipo_cells = 2;
            lipo_ok = true;
        }
        else if(between(battery_voltage,lipo_3s_low,lipo_3s_high)) {                              // ... or a 3S battery
            lipo_cells = 3;
            lipo_ok = true;
        }
        else lipo_ok = false;                                                                     // ... otherwise the battery is dead or overcharged
    }

    if(lipo_check && lipo_ok) {                                                                   // if LiPo Test OK ...
      start_timer1_motor_beep();                                                                  // ... start motor PWM for beeping
      delay(200);
      motorbeep(1,lipo_cells);                                                                    // ... beep number of detected LiPo cells
      stop_timer1();                                                                              // ... and stop PWM
    }
    
    start_timer1();                                                                               // start PWM for Motor movement
    
    rc_value_last = rc_value;                                                                     // store the latest RC value
}

// -----------------------------
// Mainprogram
// -----------------------------
void loop() {
    if(lipo_check) {                                                                              // if LiPo check is enabled ...
        if(rc_ok && lipo_ok) mot_ok = true;                                                       // check RC signal and LiPo voltage and release the motor
        else mot_ok = false;
    }
    else {                                                                                        // if LiPo check is disabled ...
        if(rc_ok) mot_ok = true;                                                                  // only check RC signal and release the motor
        else mot_ok = false;
    }
    
    while(rc_value != rc_value_last) {                                                            // only run the following commands, if RC signal varies
      
      if(rc_value > rc_max) rc_value = rc_max;                                                    // limit RC value to stored values
      if(rc_value < rc_min) rc_value = rc_min;
      
      if(mot_ok && (rc_value > (rc_middle + Hysterese_for))) {                                    // if RC signal is ok and forward (including Hysterese)
          digitalWrite(HSb, LOW);
          OCR1B = pwm_full;                                                                       // stop reverse FET B and PWM B
          delay(50);                                                                              // wait 50ms for FETs to become close
          digitalWrite(HSc, HIGH);                                                                // enable FET C and
          if(!lipo_limit) OCR1A = map(rc_value,(rc_middle + Hysterese_for),rc_max,pwm_full,0);    // map full PWM to rc_value when LiPO limit not reached
          else OCR1A = map(rc_value,(rc_middle + Hysterese_for),rc_max,pwm_full,pwm_limit);       // else map limit PWM to rc_value
      }
      else if(mot_ok && (rc_value < (rc_middle - Hysterese_rev))) {                               // if RC signal is ok and reverse (including Hysterese)
          digitalWrite(HSc, LOW);
          OCR1A = pwm_full;                                                                       // stop forward FET C and PWM C
          delay(50);                                                                              // wait 50ms for FETs to become close
          digitalWrite(HSb, HIGH);                                                                // enable FET B and
          if(reverse_half) 
            OCR1B = map(rc_value,(rc_middle - Hysterese_rev),rc_min,pwm_full,pwm_limit);          // map limit PWM when reverse half speed and LiPo limit not reached
          else {
            if(!lipo_limit) OCR1B = map(rc_value,(rc_middle - Hysterese_rev),rc_min,pwm_full,0);  // else map full PWM when LiPo limit not reached
            else OCR1B = map(rc_value,(rc_middle - Hysterese_rev),rc_min,pwm_full,pwm_limit);     // or limit PWM when reached
          }           
      }
      else {                                                                                      // if neutral or RC signal is bad ... and disable pFETs
          OCR1A = pwm_full;                                                                       // ... stop all PWM channels
          OCR1B = pwm_full;
          digitalWrite(HSb, LOW);                                                                 // ... and disable pFETs
          digitalWrite(HSc, LOW);
          
      }
      rc_value_last = rc_value;                                                                   // store latest RC signal to prevent new set of PWM
    }
    
    if(lipo_check) {                                                                              // if LiPo check is enabled
        battery_voltage = analogRead(BATTERY_PIN);                                                // get battery voltage
        switch (lipo_cells) {                                                                     // based on the numbers of LiPo cells ...
        case 2:
            if(between(battery_voltage,lipo_2s_low,lipo_2s_high)) lipo_ok = true;                 // ... check if LiPo voltage is in range
            else lipo_ok = false;
            if(battery_voltage < lipo_2s_mid) lipo_limit = true;                                  // or if already empty (< 3.5V per cell) limit speed to 50%
            break;
        case 3:
            if(between(battery_voltage,lipo_3s_low,lipo_3s_high)) lipo_ok = true;                 // and the same for 3 cells
            else lipo_ok = false;
            if(battery_voltage < lipo_3s_mid) lipo_limit = true;
            break;
        }
    }
}

// -----------------------------
// Subroutine to learn stick middle
// -----------------------------
void learn_middle() {
    while(!rc_ok) {}                                                                              // do nothing until RC signal ist stable and valid
    
    if(rc_value > 1700) learn_min_max();                                                          // if stick is on full forward start setup max & min
    
    for(int x = 0; x < 10; x++) {                                                                 // add 10 times rc_value to rc_middle (takes 800ms)
         rc_middle = rc_middle + rc_value;
         delay(80);
    }
    rc_middle = rc_middle / 10;                                                                   // devide rc_middle by 10 to get average
    if(!between(rc_middle, RCmin, RCmax)) rc_ok = false;                                          // check if value is in range
}

// -----------------------------
// Subroutine to store values in eeprom
// -----------------------------
void learn_min_max() {
    delay(500);                                                                                   // wait to start meassuring
    start_timer1_motor_beep();                                                                    // start motor PWM for beeping
    delay(500);                                                                                   // wait for PWM to become stable
    uint16_t temp_value = rc_value;                                                               // store RC value temporarly and store max to eeprom address 1
    eeprom_busy_wait();
    eeprom_write_word((uint16_t*)1,temp_value);
    motorbeep(1,1);                                                                               // beep Motor once
   
    while(!(rc_value < (temp_value - 700))) {}                                                    // wait until stick is moved down
   
    delay(1000);                                                                                  // wait again 1s to meassure
    temp_value = rc_value;                                                                        // store RC value temporarly ans store min to eeprom address 3
    eeprom_busy_wait();
    eeprom_write_word((uint16_t*)3,temp_value);
    motorbeep(1,1);                                                                               // beep Motor once
    
    rc_max = eeprom_read_word((uint16_t*)1);                                                      // get RC max and min values from eeprom
    rc_min = eeprom_read_word((uint16_t*)3);
    while(!(between(rc_value,(rc_min+300),(rc_max-300)))) {}                                      // wait until stick is in neutral position
    delay(1000);                                                                                  // wait 1s and beep Motor twice
    motorbeep(1,2);
    
    while(!((between(rc_value,(rc_max-100),(rc_max+100))) || (between(rc_value,(rc_min-100),(rc_min+100))))) {}
                                                                                                  // wait until stick is in up or down position
    delay(1000);                                                                                  // wait again 1s to meassure
    if(between(rc_value,(rc_max-100),(rc_max+100))) {                                             // check if stick is up then ...
        eeprom_busy_wait();
        eeprom_write_byte((uint8_t*)5,0);                                                         // ... store value "0" for unlimitted reverse in eeprom address 5
    }
    else {
        eeprom_busy_wait();
        eeprom_write_byte((uint8_t*)5,1);                                                         // otherwise store value "1" for limitted reverse in eeprom address 5
    }
    motorbeep(1,1);
    
    while(!(between(rc_value,(rc_min+300),(rc_max-300)))) {}                                      // wait until stick is in neutral position
    delay(1000);                                                                                  // wait 1s and beep Motor 3 times
    motorbeep(1,3);

    while(!((between(rc_value,(rc_max-100),(rc_max+100))) || (between(rc_value,(rc_min-100),(rc_min+100))))) {}
                                                                                                  // wait until stick is in up or down position
    delay(1000);                                                                                  // wait again 1s to meassure
    if(between(rc_value,(rc_max-100),(rc_max+100))) {                                             // check if stick is up then ...
        eeprom_busy_wait();
        eeprom_write_byte((uint8_t*)6,1);                                                         // ... store value "1" for LiPo protect on in eeprom address 6
    }
    else {
        eeprom_busy_wait();
        eeprom_write_byte((uint8_t*)6,0);                                                         // otherwise store value "0" for LiPo protect off in eeprom address 6
    }
    motorbeep(1,1);
    
    while(!(between(rc_value,(rc_min+300),(rc_max-300)))) {}                                      // wait until stick is in neutral position
    delay(1000);                                                                                  // wait 1s and beep Motor 2 long times
    motorbeep(0,2);
    
    stop_timer1();
}

// -----------------------------
// Subroutine to initialize Timer1
// -----------------------------
void init_timer1() {
    ICR1 = pwm_full;                                                                              // Set Timer1 max to PWM full (accordingly to 8MHz or 16MHz system clock)
    
    TCCR1A = 0;                                                                                   // clear Timer1 settings register
    TCCR1B = 0;

    bitClear(TCCR1A,WGM10);
    bitSet(TCCR1A,WGM11);
    bitSet(TCCR1B,WGM12);
    bitSet(TCCR1B,WGM13);                                                                         // Set Timer1 to FastPWM Mode 14

    bitSet(TCCR1A,COM1A0);                                                                        // enable PWM Pin D9 OC1A
    bitSet(TCCR1A,COM1A1);                                                                        // and set to inverted
    bitSet(TCCR1A,COM1B0);                                                                        // enable PWM Pin D10 OC1B
    bitSet(TCCR1A,COM1B1);                                                                        // and set to inverted
    
    TCNT1 = 0;                                                                                    // clear count register

    OCR1A = pwm_full;
    OCR1B = pwm_full;                                                                             // set PWM to 0
}

// -----------------------------
// Subroutine to start Timer1
// -----------------------------
void start_timer1() {
    ICR1 = pwm_full;                                                                              // set limit of Timer1 accordingly the MHz of the AVR

    OCR1A = pwm_full;                                                                             // set OCR register also to max, because of inverting output
    OCR1B = pwm_full;                                                                             // (means 0% PWM on the output pin)
    
    bitSet(TCCR1B,CS10);                                                                          // start Timer1 without Prescaler (8/16MHz)
    bitClear(TCCR1B,CS11);
    bitClear(TCCR1B,CS12);
}

// -----------------------------
// Subroutine to stop Timer1
// -----------------------------
void stop_timer1() {
    bitClear(TCCR1B,CS10);                                                                        // set Timer1 Prescaler to 0 (switch it off)
}

// -----------------------------
// Subroutine to start Timer1
// for motor beep
// -----------------------------
void start_timer1_motor_beep() {
    ICR1 = pwm_motor_beep;                                                                        // PWM for motor beep is only 850Hz, so set Timer1 max

    OCR1A = pwm_motor_beep;                                                                       // and also the OCR register
    OCR1B = pwm_motor_beep;                                                                       // remember the inverting output

    bitSet(TCCR1B,CS10);                                                                          // start Timer1 without Prescaler (8/16MHz)
    bitClear(TCCR1B,CS11);
    bitClear(TCCR1B,CS12);  
}

// -----------------------------
// Subroutine to beep motor 
// -----------------------------
void motorbeep(uint8_t mode, uint8_t number) {
    for(helper=0;helper<number;helper++) {                                                        // set the number of motor beeps for the loop
      digitalWrite(HSc, HIGH);                                                                    // enable HighSideFET C
      OCR1A = pwm_motor_beep_on;                                                                  // set PWM to 5%
      if(mode == 1) delay(200);                                                                   // wait 200ms for short beep
      else delay (500);                                                                           // or 500ms for long beep (will be given by variable "mode")
      digitalWrite(HSc, LOW);                                                                     // disable HighSideFET C
      OCR1A = pwm_motor_beep;                                                                     // and disable PWM
      delay(200);                                                                                 // and wait 200ms for the next beep or leave loop
    }
}

// -----------------------------
// Interrupt for RC Input
// -----------------------------
void RC_Interrupt() {
  // if rising edge store starting microseconds
  if (digitalRead(RC_PIN)) {                                                                      // check if interrupt was triggered by rising edge (start of RC impulse)
      start_micros = micros();                                                                    // then start time measurement
  }
  
  // if falling edge store stop microseconds, build average over 3 values
  else {                                                                                          // if falling edge triggered the interrupt (stop of RC impulse)
    stop_micros = micros();                                                                       // stop time measurement
    if(start_micros < stop_micros) {                                                              // check if stop time is larger than start time
      rc_temp = rc_temp + (((stop_micros - start_micros)/10)*10);                                 // limit the value to equal digits
      rc_avg_count++;
    }
    if(rc_avg_count >2) {                                                                         // and do an average calculating over 3 RC signals
      rc_value = rc_temp / 3;                                                                     // to limit differences
      rc_avg_count = 0;
      rc_temp = 0;
    }
    
    // check if value is correct
    if(between(rc_value, RCmin, RCmax)) rc_ok = true;                                             // last check if RC signal is in range
    else rc_ok = false;
  }
}
