/* Define this variable to enable the compilation 
   of the right I2C interface
 */
#ifndef ESP8266
#define ESP8266
#endif

/* Include NeoPixel library for WS2812 LEDs */
#include <Adafruit_NeoPixel.h>

/* Include basic Arduino libraries */
#include <EEPROM.h>
#include <Arduino.h>
#include <Wire.h>

/* Include ESP8266 Wifi libraries */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

/* Include JSON parser */
#include <ArduinoJson.h>

/*include OTA Libraries */
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>

/* Include current sensor library */
#include <Adafruit_INA219.h>

/* Include GRITSBot header files */
#include "GRITSBot_Main_ESP8266.h"
#include "GRITSBot_Messages.h"

/* Include GRITSBot interface definitions for I2C */
#include "include/I2CMessage.h"
#include "I2CInterface.h"

/* Include GRITSBot interface definitions for WiFi and WiFi credentials*/
#include "wirelessInterfaceESP8266.h"
#include "wifiConfig.h"

/* Include estimator and controller libraries */
#include "include/controller/controllerBase.h"
#include "include/controller/controllerTarget.h"
#include "include/estimator/estimatorBase.h"

WirelessInterfaceESP8266 wifi;
I2CInterface i2c;
Adafruit_INA219 ina219;
ControllerTarget controller;
EstimatorBase estimator;

/* Instantiate WS2812 RGB LED strip */
Adafruit_NeoPixel strip = Adafruit_NeoPixel(2, LED_PIN, NEO_GRB + NEO_KHZ800);

/* Instantiate main board */
GRITSBotMain mainboard(&wifi, &i2c, &ina219, &strip, &controller, &estimator);

/* Set velocities for forward, backward, rotate cw, rotate ccw */
float vSet[] = {0.1, -0.1,  0.0,   0.0};
float wSet[] = {0.0,  0.0, 180.0 * M_PI/180.0, -180.0 * M_PI/ 180.0};
String dir[] = {"forward", "backward", "CCW", "CW"};

unsigned long lastTransition = micros();
int mode;

void setup() {
  /* Initialize boards */
  mainboard.initialize();

  Serial.begin(115200);
  Serial.println("Mainboard initialized...");

  /* Initializing OTA */
  ArduinoOTA.begin();
  Serial.println("Ready for OTA firmware updates");
  Serial.println("Main board initialized");
  Serial.print("Version Firmware Main : "); Serial.println(mainboard.getMainBoardFirmwareVersion());
  Serial.print("Version Hardware Main : "); Serial.println(mainboard.getMainBoardHardwareVersion());
  Serial.print("Version Firmware Motor: "); Serial.println(mainboard.getMotorBoardFirmwareVersion());
  Serial.print("Version Hardware Motor: "); Serial.println(mainboard.getMotorBoardHardwareVersion());  

  /* Rainbow RGB LED animation */
  mainboard.rainbow(20);
}

void loop() {
  /* Random transitions every 2 seconds */
  if( (lastTransition + 2 * 1E6) < micros() ) {
    lastTransition = micros();

    mode = random(0, 4);
    mainboard.setVelocities(vSet[mode], wSet[mode]);
    
    Serial.print("Transition to mode: "); Serial.println(dir[mode]);
  }
  
  mainboard.setVelocities(vSet[mode], wSet[mode]);
  delay(20);
  
  /* Update wireless functions */
  yield();
  mainboard.updateWireless();
  
  /* Update measurement and data collection */
  yield();
  mainboard.updateMeasurements();

  /*OTA handling*/
  ArduinoOTA.handle();
}
