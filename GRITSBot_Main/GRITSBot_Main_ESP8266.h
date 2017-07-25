/*
 -------------------------------------------------------------------
 GRITSBot Main Board class

 METHODS:

 NOTES:

 EXAMPLES:

 Initially created by Daniel Pickem 7/18/14.
 -------------------------------------------------------------------
*/

#ifndef _GRITSBOT_MAIN_h_
#define _GRITSBOT_MAIN_h_

//------------------------------------------------------------------
// Define firmware and hardware version
//------------------------------------------------------------------
#define FIRMWARE_VERSION 20161101
#define HARDWARE_VERSION 20161020
#define FIRMWARE_ADDRESS 10
#define HARDWARE_ADDRESS 30
#define DEBUG_LEVEL 1

//------------------------------------------------------------------
// Defines
//------------------------------------------------------------------
#define LED_PIN                 4
#define STEPUP_EN_PIN           5
#define CHARGER_CONNECTION_PIN 13

//------------------------------------------------------------------
// Includes
//------------------------------------------------------------------
/* Include basic Arduino libraries */
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

/* JSON library */
#include <ArduinoJson.h>

/* Include current sensor */
#include <Adafruit_INA219.h>

/* Include NeoPixel library for WS2812 LEDs */
#include <Adafruit_NeoPixel.h>

/* Include message definitions */
#include "GRITSBot_Messages/GRITSBot_Messages.h"

/* Include I2C and wireless interfaces */
#include "GRITSBot_I2CInterface/I2CInterface.h"

/* Include GRITSBot interface definitions for WiFi and credentials */
#include "GRITSBot_WirelessInterface/wirelessInterfaceESP8266.h"
#include "GRITSBot_WiFiConfig/wifiConfig.h"

/* Include estimator and controller libraries */
#include "include/controller/controllerBase.h"
#include "include/controller/controllerTarget.h"
#include "include/estimator/estimatorBase.h"

/* Include averaging class */
#include "include/utilities/average.h"

/* Include EEPROM interface */
#include "include/utilities/EEPROM_Interface.h"

/* Include low-level ESP8266 headers */
extern "C" {
#include "c_types.h"
#include "ets_sys.h"
#include "os_type.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "smartconfig.h"
#include "lwip/opt.h"
#include "lwip/err.h"
#include "lwip/dns.h"
}

//------------------------------------------------------------------
// ENUM definitions
//------------------------------------------------------------------
enum MODE {MANUAL_MODE, CONTROLLER_MODE};

class GRITSBotMain {
  //----------------------------------------------------------------
  // Public Member Functions
  //----------------------------------------------------------------
  public:
    /* Constructors */
    GRITSBotMain(WirelessInterfaceESP8266*  radio,
                 I2CInterface*              i2c,
                 Adafruit_INA219*           ina219,
                 Adafruit_NeoPixel*         strip,
                 ControllerBase*            controller = NULL,
                 EstimatorBase*             estimator = NULL);

    /* Destructor */
    ~GRITSBotMain();

    /* Setup function */
    void initialize();

    /* Wireless UDP communication functions */
    void updateWireless();
    bool processUDPMessage();
    void sendHeartbeatMessage();
    void sendRFMessage();
    void sendErrorMessage(String error, String parameters="");
    void sendStatusMessage(String status, String parameters="");

    /* JSON communication primitives */
    void JSONSendMessage(String* fields, float* data, int len);
    void JSONSendMessage(String* fields, String* data, int len);
    void JSONSendMessage(String field, String data);
    void JSONSendMessage(String field, float data);
    void JSONSendMessage(String JSONData);

    /* Get field values from JsonObject */
    template <typename T> bool JSONGetNumber(JsonObject& root, String field, T& output);
    String JSONGetString(JsonObject& root, String field);

    /* Controls-related functions */
    void updateController();
    void updateEstimator();

    /* Visual output functions */
    void toggleLed();
    void ledOn();
    void ledOff();

    /* RGB Led output functions */
    void setLedRGB(uint8_t index, uint32_t color);
    void setLedsRGB(uint32_t color);
    void disableLedRGB(uint8_t index);
    void disableLedsRGB();
    void rainbow(uint8_t wait, uint8_t repetitions = 1);
	void setGTColor(uint16_t duration, float frequency, uint8_t color1[3], uint8_t color2[3]);
    uint32_t Wheel(byte WheelPos);

    /* Data collection functions */
    void updateMeasurements();
    bool sampleMotorBoardAverageRPS(float& rpsL, float& rpsR);
    bool sampleMotorBoardAverageTemperatures(float& tempL, float& tempR);
    bool sampleMotorBoardAverageCurrents(float& curL, float& curR);
    bool testMotorBoardI2CCommunications();

    /* Sample functions - no return value, stored in class variables */
    bool sampleMotorBoardVelocities();
    bool sampleMotorBoardRPS();
    bool sampleMotorBoardRPSMax();

    /* Status functions */
    bool isBatteryEmpty();
    bool isCharging();
    bool isCharged(bool disconnected = false);
    void chargingStatusNotification();
    void chargerConnectedNotification();
    void printChargeStatus();

    /* Power management functions */
    void enableMotorVoltage();
    void disableMotorVoltage();

    /* Sleep functions */
    void enableDeepSleep();
    void enableDeepSleep(uint32_t duration);
    void enableCurrentSensorSleep();
    void disableCurrentSensorSleep();

    /* Get functions */
    State getCurrentPosition();
    State getTargetPosition();
    float getBatteryVoltage();
    float getCurrentConsumption();
    float getStepUpVoltage();

    /* Set functions */
    void setCurrentPosition(float x, float y, float theta);
    void setTargetPosition(float x, float y, float theta);
    void setVelocities(float v, float w);
    void setVelocitiesMax(float v, float w);
    void setRPS(float rpsLeft, float rpsRight);
    void setRPSMax(float rpsMax);
    void setStepsPerRevolution(float steps);

    /* Utility functions */
    float map(float x, float inMin, float inMax, float outMin, float outMax);

    /* Versioning functions */
    bool setMainBoardFirmwareVersion(uint32_t version);
    bool setMainBoardHardwareVersion(uint32_t version);
    uint32_t getMainBoardFirmwareVersion();
    uint32_t getMotorBoardFirmwareVersion();
    uint32_t getMainBoardHardwareVersion();
    uint32_t getMotorBoardHardwareVersion();

    //--------------------------------------------------------------
    // Public Member Variables
    //--------------------------------------------------------------
    WirelessInterfaceESP8266* radio_;

    private:
    //--------------------------------------------------------------
    // Private Member Variables
    //--------------------------------------------------------------
    /* Controls-related */
    uint8_t           mode_;
    floatUnion        v_;
    floatUnion        w_;
    floatUnion        rpsLeft_;
    floatUnion        rpsRight_;
    floatUnion        rpsMax_;
    ControllerBase*   controller_;
    EstimatorBase*    estimator_;

    /* Wireless communication */
    String MACAddress_;
    int16_t ID_;

    /* I2C communication */
    I2CInterface* I2C_;
    I2CMessage I2CBuffer_;
    uint8_t I2CRequestTimeout_;

    /* Current/voltage sensing */
    Adafruit_INA219* ina219_;
    float   batteryVoltage_;
    Average current_;

    /* Charge status */
    bool chargerConnected_;
    bool chargerConnectedPrev_;

    /* Time stamps */
    uint32_t lastEstimatorUpdate_;
    uint32_t lastControllerUpdate_;
    uint32_t lastMessage_;
    uint32_t messageCounter_;
    uint32_t lastHeartbeat_;
    uint32_t lastRFMessage_;
    uint32_t lastCurrentMeasurement_;
    uint32_t lastI2CTest_;

    uint32_t lastChargeStatusCheck_;
    uint32_t lastChargedStatusMessage_;
    uint32_t lastBatteryEmpytCheck_;
    uint32_t lastDataTest_;

    /* LED parameters */
    Adafruit_NeoPixel* strip_;
};
#endif
