/*
 -------------------------------------------------------------------------------
 I2C Interface for capable of running as both master (on the GRITSBot main
 board) and slave (on the GRITSBot motor board)
 
 METHODS:

 NOTES: On the main board this class can be used in both slave and master mode
        as well.

        - I2C master (for regular operation)
        - I2C slave (for debugging)   
 
 EXAMPLES:
 
 Initially created by Daniel Pickem 6/15/15.
 -------------------------------------------------------------------------------
 */
#ifndef _I2C_INTERFACE_h_
#define _I2C_INTERFACE_h_

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <Arduino.h>
#include <Wire.h>

/* Include I2C message definitions */
#include "include/I2CMessage.h"

class I2CInterface{
    public:
      // Constructors
      I2CInterface();

      // Destructor
      ~I2CInterface();

      //--------------------------------------------------------------------------
      // Public Member Functions
      //--------------------------------------------------------------------------
#ifdef ESP8266
      void initialize(uint8_t sdaPin, uint8_t sclPin);
#else
      void initialize(uint8_t address = 0);
#endif

      /* I2C operation as master in default mode or slave mode for debugging */
      void sendMessage(uint8_t msgType, float d1, float d2);
      void sendMessage(uint8_t msgType, float d1, float d2, float d3);
      void sendMessage(uint8_t msgType, float *data, uint8_t len, uint8_t address = 2);
      void sendMessage(I2CMessage msg, uint8_t address = 2);
      bool receiveMessage(I2CMessage* msgOut, uint8_t address = 2);

      bool isMaster() {
        return isMaster_;
      }

    //--------------------------------------------------------------------------
    // Private Member Variables
    //--------------------------------------------------------------------------
    private:
      bool isMaster_;
};
#endif
