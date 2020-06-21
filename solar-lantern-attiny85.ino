#include <avr/sleep.h>
#include <avr/power.h>    // Power management
#include <avr/wdt.h>      // Watchdog timer
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

// 10 second test
//long cycles = 15;
// 4.5 hours (6intervals*60mins*4.5hrs)
long cycles = 1620;

long readVcc() {
  ADMUX = _BV(MUX3) | _BV(MUX2);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}



// watchdog interrupt
ISR (WDT_vect)
{
  wdt_disable();  // disable watchdog
}  // end of WDT_vect

void resetWatchdog ()
{
  // clear various "reset" flags
  MCUSR = 0;
  // allow changes, disable reset, clear existing interrupt
  WDTCR = bit (WDCE) | bit (WDE) | bit (WDIF);
  // set interrupt mode and an interval (WDE must be changed from 1 to 0 here)
  WDTCR = bit (WDIE) | bit (WDP3) | bit (WDP1) | bit (WDP0);    // set WDIE, and 2 seconds delay
  // pat the dog
  wdt_reset();
}  // end of resetWatchdog

void setup()
{
  resetWatchdog ();  // do this first in case WDT fires
  for (int i = 1; i < 5; i++) {
    pinMode(i, OUTPUT);
  }
  //add input on solar voltage
  pinMode(4, INPUT);

}

void loop()
{
  //check the solar panel voltage
  int sensorValue = analogRead(A2);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float solarVoltage = sensorValue * (5.0 / 1023.0);
  long voltage = readVcc();

  //check if its been 4.5 hours, low voltage or if its daytime
  if ( cycles < 10 || voltage < 3200 || solarVoltage >= 0.61) {
    //      reset the cycles if its daytime
    if ( solarVoltage >= 0.61) {
      cycles = 1620;
    }
    //  daytime so turn off the lights and go to sleep
    digitalWrite(3, LOW);
    digitalWrite(0, LOW);
    goToSleep ();
  } else {
    //  its night, turn on the main light
    digitalWrite(3, HIGH);
    //test if the voltage is low, turn on red light
    if (voltage < 3400) {
      analogWrite(0, 50);
    } else {
      digitalWrite(0, LOW);
    }

    delay(9998);
    cycles --;
  }



  //testing it cycling...
  //  digitalWrite(0, HIGH);
  //  delay(100);
  //  digitalWrite(0, LOW);

}

void goToSleep ()
{

  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  static byte prevADCSRA = ADCSRA;
  ADCSRA = 0;
  power_all_disable ();  // power off ADC, Timer 0 and 1, serial interface
  noInterrupts ();       // timed sequence coming up
  resetWatchdog ();      // get watchdog ready
  sleep_enable ();       // ready to sleep
  interrupts ();         // interrupts are required now
  sleep_cpu ();          // sleep
  sleep_disable ();      // precaution
  power_all_enable ();   // power everything back on
  ADCSRA = prevADCSRA;
}  // end of goToSleep
