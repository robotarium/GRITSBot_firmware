/* Define this variable to enable the compilation
   of the right I2C interface
 */
#define ESP8266

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
//#include <Adafruit_INA219.h>

/* Include GRITSBot header files */
#include "GRITSBot_Main/GRITSBot_Main_ESP8266.h"
#include "GRITSBot_Messages/GRITSBot_Messages.h"

/* Include GRITSBot interface definitions for I2C */
#include "GRITSBot_I2CInterface/include/I2CMessage.h"
#include "GRITSBot_I2CInterface/I2CInterface.h"

/* Include GRITSBot interface definitions for WiFi and WiFi credentials*/
#include "GRITSBot_WirelessInterface/wirelessInterfaceESP8266.h"
#include "GRITSBot_WiFiConfig/wifiConfig.h"

/* Include estimator and controller libraries */
#include "GRITSBot_Main/include/controller/controllerBase.h"
#include "GRITSBot_Main/include/controller/controllerTarget.h"
#include "GRITSBot_Main/include/estimator/estimatorBase.h"

/* Instantiate Wifi UDP client */
WirelessInterfaceESP8266 wifi;

/* Instantiate I2C client */
I2CInterface i2c;

/* Instantiate currenet sensor */
Adafruit_INA219 ina219;

/* Instantiate controller and estimator */
ControllerTarget controller;
EstimatorBase estimator;

/* Instantiate main board */
GRITSBotMain mainboard(&wifi, &i2c, &ina219, &controller, &estimator);

/* **********************
 *     SETUP FUNCTION
 * **********************/
void setup() {
  Serial.begin(115200);
  Serial.println("Main board started");

  /* Initialize boards */
  mainboard.initialize();

  /* Initializing OTA */
  ArduinoOTA.begin();
  Serial.println("Ready for OTA firmware updates");
  Serial.println("Main board initialized");
}

/* ********************
 *     MAIN LOOP
 * ********************/
void loop() {
  /* Update wireless function takes care of all message processing
   * and sends out heartbeat message every second
   */
  yield();
  mainboard.updateWireless();

  /* Update controller */
  yield();
  mainboard.updateController();

  /* Update measurement and data collection */
  yield();
  mainboard.updateMeasurements();

  /*OTA handling*/
  ArduinoOTA.handle();
}
