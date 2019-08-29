/* -----------------------------------------------------------------------
** ArduSpeedHF 30A V0.1 - Speedcontroller for brushed Motors based on 
** SimonK 30A ESC at aliexpress.com (Hobbypower 30A)
**
** based on the "MiniCore" https://github.com/MCUdude/MiniCore
** =======================================================================
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
** Outputs:
** ---------
** D5 - pFET A
** D10 - nFET A
** D4 - pFET B
** D9 - nFET B
** D3 - pFET C
** D8 - nFET C
**
** EEPROM data:
** -------------
** 1 - max RC value
** 3 - min RC value
** 5 - 50% reverse (1=true / 0=false)
** 6 - lipo check (1=on / 0=off)
**
** LiPo analog read values
** ------------------------
** 2S low (6,6V) = 120
** 2S mid (7,4V) = 135
** 2S high (8,4V) = 160
** 3S low (9,9V) = 180
** 3S mid (11,1V) = 205
** 3S high (12,6V) = 235
*  LiPo Limit (13V) = 250
**
** Author: M.Stoffers
** Year: 2019
** License: CC (BY|NC|SA)
** -------------------------------------------------------------------------*/
#include <avr/eeprom.h>

// -----------------------------
// User-defines
// -----------------------------
#define Hysterese_for 50       // Hysterese for stick from middle position to motor starts forward (deadzone of stick)
#define Hysterese_rev 50       // Hysterese for stick from middle position to motor starts reverse (deadzone of stick)

// -----------------------------
// System-defines
// -----------------------------
#define ApFET 5
#define AnFET 10
#define BpFET 4
#define BnFET 9
#define CpFET 3
#define CnFET 8

#define battery_pin A0
#define MuxA A5
#define MuxB A4

#define RC_PIN 2

#define lipo_2s_low 120
#define lipo_2s_mid 135
#define lipo_2s_high 160
#define lipo_3s_low 180
#define lipo_3s_mid 205
#define lipo_3s_high 235
#define lipo_limit 250

#define between(x, a, b)  (((a) <= (x)) && ((x) <= (b)))
// -----------------------------
// declares
// -----------------------------

// -----------------------------
// variables
// -----------------------------
uint8_t helper;
uint8_t pin_array[10] {ApFET, AnFET, BpFET, BnFET, CpFET, CnFET, MuxA, MuxB};

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

uint16_t pwm_forward = 1023;                                              // set forward to 100%
uint16_t pwm_reverse = 1023;                                              // set reverse to 100%
uint16_t pwm_limit = 512;                                                 // set forward limit to 50%
boolean mot_ok = false;
boolean timer1_running = false;
volatile boolean forward = false;
volatile boolean reverse = false;
volatile boolean full = false;
boolean reverse_half = false;
boolean full_ok = true;

uint16_t battery_voltage = 0;
uint8_t lipo_cells = 0;
boolean lipo_ok = false;
boolean lipo_check = false;

// -----------------------------
// Setup
// -----------------------------
void setup() {
    for(helper = 0; helper < 6; helper++) {                                                 // set all MosFET Pins ...
      pinMode(pin_array[helper], OUTPUT);                                                   // ... as Output ...
      digitalWrite(pin_array[helper], LOW);                                                 // ...  and to LOW
    }
    for(helper = 6; helper < 8; helper++) {                                                 // read all analog pins to ensure that the pin
      analogRead(pin_array[helper]);                                                        // direction is set correctly
    }
    attachInterrupt(digitalPinToInterrupt(RC_PIN), RC_Interrupt, CHANGE);                   // start interrupt for RC1 meassuring
    delay(500);

    init_timer1();
    init_timer2();
        
    learn();                                                                                // learn stick middle and setup min & max if wanted
    
    rc_max = eeprom_read_word((uint16_t*)1);                                                // read EEPROM value for RC max
    if(!between(rc_max, 900, 2100)) rc_max = 2000;                                          // and check if valid
    
    rc_min = eeprom_read_word((uint16_t*)3);                                                // read EEPROM value for RC min
    if(!between(rc_min, 900, 2100)) rc_min = 1000;                                          // and check if valid

    if(eeprom_read_byte((uint8_t*)5) == 1) reverse_half = true;                             // check if reverse limit is set and set reverse value to 50%
    
    if(eeprom_read_byte((uint8_t*)6) == 1) lipo_check = true;                               // check if lipo check is set to on
    else lipo_check = false;                                                                // else switch off
        
    if(lipo_check) {                                                                        // if LiPo check is enabled ...
        battery_voltage = analogRead(battery_pin);
        
        if(between(battery_voltage,lipo_2s_low,lipo_2s_high)) {                             // ... check if it is a 2S battery
            lipo_cells = 2;
            lipo_ok = true;
        }
        else if(between(battery_voltage,lipo_3s_low,lipo_3s_high)) {                        // ... or a 3S battery
            lipo_cells = 3;
            lipo_ok = true;
        }
        else lipo_ok = false;                                                               // ... otherwise the battery is dead or overcharged
    }
    
    if(lipo_ok) {                                                                           // if LiPo Test OK ...
      for(helper = 0; helper < lipo_cells; helper++) {                                      // ... beep number of detected LiPo cells
        motorbeepshort();
        delay(250);
      }
    }
    
    rc_value_last = rc_value;                                                               // store the latest RC value
}

// -----------------------------
// Mainprogram
// -----------------------------
void loop() {
    if(lipo_check) {
        if(rc_ok && lipo_ok) mot_ok = true;
        else mot_ok = false;
    }
    else {
        if(rc_ok) mot_ok = true;
        else mot_ok = false;
    }
    
    while(rc_value != rc_value_last) {
      
      if(rc_value > rc_max) rc_value = rc_max;                                                // limit RC value to stored values
      if(rc_value < rc_min) rc_value = rc_min;
      
      if(mot_ok && (rc_value > (rc_middle + Hysterese_for))) {                                // if RC signal is ok and forward (including Hysterese)
          digitalWrite(AnFET, LOW);
          digitalWrite(BpFET, LOW);                                                           // stop AnFET & BpFET
          reverse = false;
          forward = true;                                                                     // set to forward
          delay(2);                                                                           // wait 2ms for FETs to become close
          digitalWrite(BnFET, HIGH);                                                          // enable BnFET and
          OCR1A = map(rc_value,(rc_middle + Hysterese_for),rc_max,50,pwm_forward);            // map PWM forward to 100 - 03FFh (Top of Timer1)
          if(full_ok && (rc_value >= (rc_max - 10))) full = true;
          else full = false;
          if(!timer1_running) start_timer1();
      }
      else if(mot_ok && (rc_value < (rc_middle - Hysterese_rev))) {                           // if RC signal is ok and reverse (including Hysterese)
          digitalWrite(BnFET, LOW);
          digitalWrite(ApFET, LOW);                                                          // stop BnFET & ApFET
          reverse = true;                                                                     // set to reverse
          forward = false;
          delay(2);                                                                           // wait 2ms for FETs to become close
          digitalWrite(AnFET, HIGH);                                                          // enable AnFET and
          if(reverse_half) OCR1A = map(rc_value,(rc_middle - Hysterese_rev),rc_min,50,pwm_limit);  // map PWM from 10% - 50% when reverse half speed
          else OCR1A = map(rc_value,(rc_middle - Hysterese_rev),rc_min,50,pwm_reverse);       // else map PWM from 10% - 100%
          if(full_ok && !reverse_half && (rc_value <= (rc_min + 10))) full = true;
          else full = false;
          if(!timer1_running) start_timer1();
      }
      else {                                                                                  // if neutral or RC signal is bad stop all PWM channels and disable nFETs
          stop_timer1();
          digitalWrite(ApFET, LOW);                                                          // clear all MosFET's
          digitalWrite(BnFET, LOW);
          digitalWrite(BpFET, LOW);
          digitalWrite(AnFET, LOW);
      }
      rc_value_last = rc_value;                                                               // store latest RC signal to prevent new set of PWM
    }
    
    if(lipo_check) {
        battery_voltage = analogRead(battery_pin);
        switch (lipo_cells) {
        case 2:
            if(between(battery_voltage,lipo_2s_low,lipo_2s_high)) lipo_ok = true;
            else lipo_ok = false;
            if(battery_voltage < lipo_2s_mid) {
                pwm_forward = pwm_limit;
                pwm_reverse = pwm_limit;
                full_ok = false;
            }
            break;
        case 3:
            if(between(battery_voltage,lipo_3s_low,lipo_3s_high)) lipo_ok = true;
            else lipo_ok = false;
            if(battery_voltage < lipo_3s_mid) {
                pwm_forward = pwm_limit;
                pwm_reverse = pwm_limit;
                full_ok = false;
            }
            break;
        }
    }
}

// -----------------------------
// Subroutine to learn stick middle
// -----------------------------
void learn() {
    while(!rc_ok) {}                                   // do nothing until RC signal ist stable and valid
    
    if(rc_value > 1700) store();                       // if stick is on full forward start setup max & min
    
    for(int x = 0; x < 10; x++) {                      // add 10 times rc_value to rc_middle (takes 800ms)
         rc_middle = rc_middle + rc_value;
         delay(80);
    }
    
    rc_middle = rc_middle / 10;                        // devide rc_middle by 10 to get average
    if(!between(rc_middle, 900, 2100)) rc_ok = false;  // check if value is in range
}

// -----------------------------
// Subroutine to store values in eeprom
// -----------------------------
void store() {
    delay(1000);                                       // wait 1s to start meassuring
    uint16_t temp_value = rc_value;                    // store RC value temporarly and store max to eeprom address 1
    eeprom_busy_wait();
    eeprom_write_word((uint16_t*)1,temp_value);
    motorbeepshort();                                  // beep Motor once
   
    while(!(rc_value < (temp_value - 700))) {}         // wait until stick is moved down
   
    delay(1000);                                       // wait again 1s to meassure
    temp_value = rc_value;                             // store RC value temporarly ans store min to eeprom address 3
    eeprom_busy_wait();
    eeprom_write_word((uint16_t*)3,temp_value);
    motorbeepshort();                                  // beep Motor once
    
    rc_max = eeprom_read_word((uint16_t*)1);           // get RC max and min values from eeprom
    rc_min = eeprom_read_word((uint16_t*)3);
    while(!(between(rc_value,(rc_min+300),(rc_max-300)))) {}
                                                       // wait until stick is in neutral position
    delay(1000);                                       // wait 1s and beep Motor twice
    motorbeepshort();
    delay(200);
    motorbeepshort();
    
    while(!((between(rc_value,(rc_max-100),(rc_max+100))) || (between(rc_value,(rc_min-100),(rc_min+100))))) {}
                                                       // wait until stick is in up or down position
    delay(1000);                                       // wait again 1s to meassure
    if(between(rc_value,(rc_max-100),(rc_max+100))) {  // check if stick is up then ...
        eeprom_busy_wait();
        eeprom_write_byte((uint8_t*)5,0);              // ... store value "0" for unlimitted reverse in eeprom address 5
    }
    else {
        eeprom_busy_wait();
        eeprom_write_byte((uint8_t*)5,1);              // otherwise store value "1" for limitted reverse in eeprom address 5
    }
    motorbeepshort();
    
    while(!(between(rc_value,(rc_min+300),(rc_max-300)))) {}
                                                       // wait until stick is in neutral position
    delay(1000);                                       // wait 1s and beep Motor 3 times
    motorbeepshort();
    delay(200);
    motorbeepshort();
    delay(200);
    motorbeepshort();

    while(!((between(rc_value,(rc_max-100),(rc_max+100))) || (between(rc_value,(rc_min-100),(rc_min+100))))) {}
                                                       // wait until stick is in up or down position
    delay(1000);                                       // wait again 1s to meassure
    if(between(rc_value,(rc_max-100),(rc_max+100))) {  // check if stick is up then ...
        eeprom_busy_wait();
        eeprom_write_byte((uint8_t*)6,1);              // ... store value "1" for LiPo protect on in eeprom address 6
    }
    else {
        eeprom_busy_wait();
        eeprom_write_byte((uint8_t*)6,0);              // otherwise store value "0" for LiPo protect off in eeprom address 6
    }
    motorbeepshort();
    
    while(!(between(rc_value,(rc_min+300),(rc_max-300)))) {}
                                                       // wait until stick is in neutral position
    delay(1000);                                       // wait 1s and beep Motor 3 times
    motorbeeplong();
    delay(200);
    motorbeeplong();
}

// -----------------------------
// Subroutine to initialize Timer1
// -----------------------------
void init_timer1() {
    bitSet(TCCR1A,WGM10);
    bitSet(TCCR1A,WGM11);
    bitSet(TCCR1B,WGM12);                               // Set Timer 1 to FastPWM and 10bit
}

// -----------------------------
// Subroutine to initialize Timer2
// -----------------------------
void init_timer2() {
    OCR2 = 20;
}

// -----------------------------
// Subroutine to start Timer1
// -----------------------------
void start_timer1() {
    bitSet(TCCR1B,CS10);                                // Start Timer 1 without Prescaler (16MHz)
    bitClear(TCCR1B,CS11);
    bitClear(TCCR1B,CS12);
    bitSet(TIMSK,OCIE1A);
    bitSet(TIMSK,TOIE1);                                // and enable Interrupts for Overflow and Compare
    timer1_running = true;
}

// -----------------------------
// Subroutine to stop Timer1
// -----------------------------
void stop_timer1() {
    bitClear(TCCR1B,CS10);                              // Stop Timer 1
    bitClear(TIMSK,OCIE1A);
    bitClear(TIMSK,TOIE1);                              // and disable Interrupts
    timer1_running = false;
}

// -----------------------------
// Subroutine to start Timer2
// -----------------------------
void start_timer2() {
    bitSet(TIMSK,OCIE2);
    bitSet(TIMSK,TOIE2);                                // enable Interrupts for Overflow and Compare
}

// -----------------------------
// Subroutine to stop Timer2
// -----------------------------
void stop_timer2() {
    bitClear(TIMSK,OCIE2);
    bitClear(TIMSK,TOIE2);                              // and disable Interrupts
    digitalWrite(ApFET, LOW);    
}

// -----------------------------
// Subroutine to beep motor shortly
// -----------------------------
void motorbeepshort() {
    start_timer2();                                     // start Timer2 PWM
    digitalWrite(BnFET, HIGH);                          // and enable BnFET
    delay(200);                                         // wait 200ms
    stop_timer2();                                      // stop Timer2 PWM (silence)
    digitalWrite(BnFET, LOW);                           // and disable BnFET
}

// -----------------------------
// Subroutine to beep motor long
// -----------------------------
void motorbeeplong() {
    start_timer2();                                     // start Timer2 PWM
    digitalWrite(BnFET, HIGH);                          // and enable BnFET
    delay(500);                                         // wait 500ms
    stop_timer2();                                      // stop Timer2 PWM (silence)
    digitalWrite(BnFET, LOW);                           // and disable BnFET
}

// -----------------------------
// Interrupt for RC Input
// -----------------------------
void RC_Interrupt() {

    // if rising edge store starting microseconds
    if (digitalRead(RC_PIN)) {
        start_micros = micros();
    }
    // if falling edge store stop microseconds, build average over 3 values
    else {
        stop_micros = micros();
        if(start_micros < stop_micros) {
            rc_temp = rc_temp + (((stop_micros - start_micros)/10)*10);
            rc_avg_count++;
        }
        if(rc_avg_count >2) {
            rc_value = rc_temp / 3;
            rc_avg_count = 0;
            rc_temp = 0;
        }
        // check if value is correct
        if(between(rc_value, 900, 2100)) rc_ok = true;
        else rc_ok = false;
    }
}

// -----------------------------
// Interrupt for Timer1 Overflow
// -----------------------------
ISR(TIMER1_OVF_vect){
    if(forward && !reverse) digitalWrite(ApFET, HIGH);
    if(reverse && !forward) digitalWrite(BpFET, HIGH);
}

// -----------------------------
// Interrupt for Timer2 Overflow
// -----------------------------
ISR(TIMER2_OVF_vect){
    digitalWrite(ApFET, HIGH);
}

// -----------------------------
// Interrupt for Timer1 Compare Match
// -----------------------------
ISR(TIMER1_COMPA_vect){
    if(forward && !reverse && !full) digitalWrite(ApFET, LOW);
    if(reverse && !forward && !full) digitalWrite(BpFET, LOW);
}

// -----------------------------
// Interrupt for Timer2 Compare Match
// -----------------------------
ISR(TIMER2_COMP_vect){
    digitalWrite(ApFET, LOW);
}
