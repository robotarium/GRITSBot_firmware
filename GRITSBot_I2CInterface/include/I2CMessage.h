/*
 -------------------------------------------------------------------------------
 I2C Message class used in the I2C interface class
 
 METHODS:

 NOTES: 
 
 EXAMPLES:
 
 Initially created by Daniel Pickem 8/27/16.
 -------------------------------------------------------------------------------
 */

#ifndef _I2C_MESSAGE_h_
#define _I2C_MESSAGE_h_

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif

#define MAX_I2C_MSG_LENGTH  3

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <Arduino.h>

union floatUnion {
  uint8_t b[4];
  float fval;
};

class I2CMessage{
  public:
    I2CMessage() {
      msgType_ = 0;
      for (int i = 0; i < MAX_I2C_MSG_LENGTH; i++) {
        data_[i].fval = 0.0;
      }
    };

    I2CMessage(uint8_t msgType, float* data, uint8_t len) {
      msgType_ = msgType;
      for (int i = 0; i < min((uint8_t)MAX_I2C_MSG_LENGTH, len); i++) {
        data_[i].fval = data[i];
      }
    };

    ~I2CMessage() {};

    void clear() {
      for (int i = 0; i < MAX_I2C_MSG_LENGTH; i++){
        data_[i].fval = 0;
      }
    }

    /* Print function
     * NOTE: Overloading the output stream operator would only work on 
     *       the host computer, but not on a microcontroller that relies 
     *       on serial communication.
     * NOTE: This function relies on an instantiated Serial connection
     *       (using Serial.begin(baudrate);)
     */
    void print() {
      Serial.print("Msg. type / data: "); 
      Serial.print(msgType_);                
      Serial.print(" / ");

      for (uint8_t i = 0; i < MAX_I2C_MSG_LENGTH; i++) {
        Serial.print(data_[i].fval); Serial.print(',');
      }  
      Serial.println();
    }

  public:
    uint8_t msgType_;
    floatUnion data_[MAX_I2C_MSG_LENGTH];
};
#endif
