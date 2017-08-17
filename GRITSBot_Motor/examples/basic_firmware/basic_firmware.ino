#include <GRITSBot_Motor.h>

#include <GRITSBot_Motor.h>

#include <GRITSBot_Motor.h>

#include <GRITSBot_Messages.h>

#include <GRITSBot_Motor.h>

/* Include Arduino libraries */
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

/* Include GRITSBot libraries */
#include "GRITSBot_Motor.h"
#include "GRITSBot_Messages.h"

/* Include GRITSBot interface definitions for I2C */
#include "include/I2CMessage.h"
#include "I2CInterface.h"

/* Include GRITSBot utilities */
#include "include/utilities/average.h"

#undef ESP8266
#define F_CPU 8000000UL

/* Forward declare I2C events */
void requestEvent();
void receiveEvent(int nBytes);

/* Instantiate GRITSBot motor board */
GRITSBotMotor motorboard;

void setup() {
  /* Initialize motorboard */
  motorboard.initialize();
  motorboard.ledsOn();

  /* Register I2C events */
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  Serial.begin(115200);
  Serial.println("Started motor board");
}

void loop() {
  /* Update motors */
  motorboard.step();
}

void requestEvent() {
  motorboard.requestEvent();
}

void receiveEvent(int nBytes) {
  motorboard.receiveEvent();
}

/* Watchdog interrupt routine */
ISR(WDT_vect) {
  wdt_disable();
}
