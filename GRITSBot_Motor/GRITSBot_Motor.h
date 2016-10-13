/*
 -------------------------------------------------------------------------------
 GRITSBot Motor Board class
 
 METHODS:

 NOTES:
 
 EXAMPLES:
 
 Initially created by Daniel Pickem 7/18/14.
 -------------------------------------------------------------------------------
 */

#ifndef _GRITSBOT_MOTOR_h_
#define _GRITSBOT_MOTOR_h_

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

/* Include message protocol and I2C interface headers */
#include "GRITSBot_Messages.h"
#include "I2CInterface.h"

/* Include utility headers */
#include "include/utilities/average.h"
#include "include/utilities/EEPROM_Interface.h"

//------------------------------------------------------------------------------
// CPU frequency (8 MHz) 
//------------------------------------------------------------------------------
#define F_CPU 8000000UL

//------------------------------------------------------------------------------
// Macros 
//------------------------------------------------------------------------------
#define sbi(a, b) (a) |= (1 << (b))
#define cbi(a, b) (a) &= ~(1 << (b))

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define I2C_TX_LEN_MOTOR    3
#define STEPS_PER_REVOLUTION 60

//------------------------------------------------------------------
// Define firmware version
//------------------------------------------------------------------
#define FIRMWARE_VERSION 13102016
#define FIRMWARE_ADDRESS 10

//------------------------------------------------------------------------------
// Define motor pins
//------------------------------------------------------------------------------
/* Yellow Circuithub robots are     V5 */
/* Green Seeedstudio robots are     V4 */
/* Purple Oshpark robots are mostly V3 */
#define VERSION 5

#if VERSION == 5
/* Yellow Circuithub robots are V5 */
/* LED pins */
#define LED_LEFT_PORT   PORTD
#define LED_LEFT_PIN        2
#define LED_RIGHT_PORT  PORTD
#define LED_RIGHT_PIN       4

/* Main board reset pin */
#define MAINBOARD_RESET_PORT   PORTD
#define MAINBOARD_RESET_PIN        3

/* Motor 1 pin and port definitions */
#define M11_PORT PORTB
#define M12_PORT PORTB
#define M13_PORT PORTB
#define M14_PORT PORTD
// TODO: for version 5 fix nextStepLeft/Right to move in the correct way fwd, bwd, ccw, cw
#define M11_PIN 2 // swapped to ensure right directions
#define M12_PIN 1 // swapped
#define M13_PIN 0
#define M14_PIN 7

/* Motor 2 pin and port definitions */
#define M21_PORT PORTD
#define M22_PORT PORTD
#define M23_PORT PORTB
#define M24_PORT PORTB
#define M21_PIN 6 // swapped
#define M22_PIN 5 // swapped
#define M23_PIN 7
#define M24_PIN 6

#else
/* Green Seeedstudio robots are V4 */
/* Two LED pins */
#define LED_LEFT_PORT   PORTD
#define LED_LEFT_PIN        4
#define LED_RIGHT_PORT  PORTD
#define LED_RIGHT_PIN       4

/* Main board reset pin not available for V4 - define as same as LED pin */
#define MAINBOARD_RESET_PORT   PORTD
#define MAINBOARD_RESET_PIN        4

/* v4.0: Motor 1 pin and port definitions */
#define M11_PORT PORTB
#define M12_PORT PORTB
#define M13_PORT PORTB
#define M14_PORT PORTD
#define M11_PIN 1
#define M12_PIN 2
#define M13_PIN 0
#define M14_PIN 7

/* v4.0: Motor 2 pin and port definitions */
#define M21_PORT PORTD
#define M22_PORT PORTD
#define M23_PORT PORTB
#define M24_PORT PORTB
#define M21_PIN 5
#define M22_PIN 6
#define M23_PIN 7
#define M24_PIN 6
#endif

class GRITSBotMotor {
    public:
    //--------------------------------------------------------------------------
    // Lifecycle
    //--------------------------------------------------------------------------
    /* Constructors */
    GRITSBotMotor();

    /* Destructor */
    ~GRITSBotMotor();

    //--------------------------------------------------------------------------
    // Public Member Functions
    //--------------------------------------------------------------------------
    /* Setup functions */
    /* Note that address = 2 is the default address for the motor board 
     * The main board expects that address for communication.
     */
    void initialize(uint8_t address = 2);

    /* Main motor function */
    void step();
    void stopMotors();

    /* I2C-related functions */
    void requestEvent();
    void receiveEvent();
    void processI2CMessage(I2CMessage* msg);

    /* Setters */
    void setVelocities(float v, float w);
    void setVelocitiesMax(float vMax, float wMax);
    void setRPS(float rpsLeft, float rpsRight);
    void setRPSMax(float rpsMax);
    void setStepsPerRevolution(uint16_t steps);

    /* LED functions */
    void toggleLeds();
    void toggleLedLeft();
    void toggleLedRight();
    void ledsOn();
    void ledsOff();
    void ledOnLeft();
    void ledOnRight();
    void ledOffLeft();
    void ledOffRight();
    
    /* Status functions */
    bool isMaster() { return i2c_.isMaster(); };

    /* Sleep mode functions */
    void enableDeepSleep();
    void enableDeepSleep(uint32_t sec);
    void sleep1Sec();
    void sleep8Sec();
    void sleepNSec(const byte interval);
    void resetMainBoard();

    /* Data collection functions */
    void collectData();

    /* Versioning functions */
    bool setVersion(uint32_t version);
    uint32_t getVersion();

    //--------------------------------------------------------------------------
    // Public Member Variables
    //--------------------------------------------------------------------------
    public:
    /* I2C communication-related variables */
    I2CInterface i2c_;
    I2CMessage I2CBuffer_;

    //--------------------------------------------------------------------------
    // Private Member Functions
    //--------------------------------------------------------------------------
    private:
    /* Motor control */
    void stepLeft();
    void stepLeft(int step);
    void stepRight();
    void stepRight(int step);

    void nextStepLeft();
    void nextStepRight();

    bool isStopped();
    void stopMotorLeft();
    void stopMotorRight();

    void saturateVelocities();
    float map(float x, float inMin, float inMax, float outMin, float outMax);

    //--------------------------------------------------------------------------
    // Private Member Variables
    //--------------------------------------------------------------------------
    /* Wheel and robot parameters 
     * 		wheel radius ... 5 mmm
     * 		wheel base   ... 35 mm
     */
    float rpsMax_;
    const float rWheel_ = 0.005;
    const float cWheel_ = 0.0314;
    const float rTrack_ = 0.0175;
    const float cTrack_ = 0.1100;
    uint16_t    stepsPerRevolution_;

    /* Velocities */
    float rpsLeft_;
    float rpsRight_;
    float v_;
    float w_;
    float vMax_;
    float wMax_;
    
    /* Motor control parameters */
    unsigned long delayLeft_;
    unsigned long delayRight_;
    uint8_t curStepLeft_;
    uint8_t curStepRight_;
    unsigned long lastStepLeft_;
    unsigned long lastStepRight_;

    /* Deep sleep parameters */
    uint32_t sleepDuration_;
    bool     sleepNow_;

    /* Data collection */
    Average rpsLeftAvg_;
    Average rpsRightAvg_;
    Average tempLeftAvg_;
    Average tempRightAvg_;
    Average currentLeftAvg_;
    Average currentRightAvg_;

    /* LED parameters */
    bool ledStateLeft_;
    bool ledStateRight_;
};
#endif
