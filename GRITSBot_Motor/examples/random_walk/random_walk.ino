/* Include Arduino libraries */
#include <Arduino.h>
#include <Wire.h>

/* Include GRITSBot libraries */
#include "GRITSBot_Motor.h"
#include "GRITSBot_Messages.h"

/* Include GRITSBot interface definitions for I2C */
#include "include/I2CMessage.h"
#include "I2CInterface.h"

/* Include GRITSBot utilities */
#include "include/utilities/average.h"

#define F_CPU 8000000UL

/* Forward declare I2C events */
void requestEvent();
void receiveEvent(int nBytes);

/* Instantiate GRITSBot motor board */
GRITSBotMotor motorboard;

/* Set velocities for forward, backward, rotate cw, rotate ccw */
float vSet[] = {0.1, -0.1,  0.0,   0.0};
float wSet[] = {0.0,  0.0, 360.0, -360.0};
//float vSet[] = {0.1, 0.0};
//float wSet[] = {0.0, 360.0};

unsigned long lastTransition = micros();

void setup() {
  /* Initialize motorboard */
  motorboard.initialize();
  motorboard.ledsOn();
  
  /* Register I2C events */
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  /* Set initial velocities v, w */
  motorboard.setVelocities(-0.1, 0.0);
}

void loop() {
  /* Update motors */
  motorboard.step();

  /* Random transitions every 1 seconds */
  if( (lastTransition + 1 * 1E6) < micros() ) {
    lastTransition = micros();

    int mode = random(0, 4);
    motorboard.setVelocities(vSet[mode], wSet[mode]);
    motorboard.toggleLeds();
  }
}

void requestEvent() {
  motorboard.requestEvent();  
}

void receiveEvent(int nBytes) {
  motorboard.receiveEvent();  
}
