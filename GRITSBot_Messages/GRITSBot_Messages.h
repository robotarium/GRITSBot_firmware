/*
 -------------------------------------------------------------------------------
 GRITSBot Message Definitions
 
 METHODS: None

 NOTES: These message type definitions govern the full operation of the
        GRITSBot within the Robotarium. 
 
 EXAMPLES: None
 
 Initially created by Daniel Pickem 4/3/15.
 -------------------------------------------------------------------------------
 */

#ifndef _GRITSBOT_MESSAGES_h_
#define _GRITSBOT_MESSAGES_h_

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Define message types
//------------------------------------------------------------------------------
/* Individual set messages */
#define MSG_SET_CURRENT_POSE      1
#define MSG_SET_TARGET_POSE       2
#define MSG_SET_VELOCITIES        3
#define MSG_SET_VELOCITIES_MAX    4
#define MSG_SET_RPS               5
#define MSG_SET_RPS_MAX           6
#define MSG_SET_LED_RGB           7
#define MSG_SET_ALL_LED_RGB       8
#define MSG_SET_EEPROM_CELL       9
#define MSG_SET_MODE              10
#define MSG_SET_STEPS_PER_REV     11
#define MSG_UPDATE_ESTIMATOR      12
#define MSG_UPDATE_DYNAMICS       13

/* Individual get messages */
#define MSG_GET_CURRENT_POSE      20
#define MSG_GET_TARGET_POSE       21
#define MSG_GET_BATT_VOLT         22
#define MSG_GET_BATT_EMPTY        23
#define MSG_GET_DISTANCE          24
#define MSG_GET_DISTANCES         25
#define MSG_GET_ACCELEROMETER     26
#define MSG_GET_GYROSCOPE         27
#define MSG_GET_VELOCITIES        28
#define MSG_GET_VELOCITIES_MAX    29
#define MSG_GET_RPS               30
#define MSG_GET_RPS_MAX           31
#define MSG_GET_LED_RGB           32 
#define MSG_GET_EEPROM_CELL       33
#define MSG_GET_STATE             34
#define MSG_GET_BATTERY_LEVEL     35
#define MSG_GET_NEIGHBORS         36
#define MSG_GET_FIRMWARE_VERSION  37
#define MSG_GET_HARDWARE_VERSION  38

/* Get average performance messages */
#define MSG_GET_AVG_RPS           50 
#define MSG_GET_AVG_TEMPERATURES  51 
#define MSG_GET_AVG_CURRENTS      52 

/* Status and management messages */
#define MSG_ERASE_EEPROM_CELL     42 
#define MSG_FORMAT_EEPROM         43
#define MSG_HOST_IP               44
#define MSG_DEEP_SLEEP            45
#define MSG_CHARGED               46
#define MSG_DISCHARGED            47
#define MSG_CHARGING              48
#define MSG_STATUS                90
#define MSG_ERROR                 91
#define MSG_RF_DATA               92
#define MSG_DEBUG                 93
#define MSG_REMATCH               94
#define MSG_ECHO                  95
#define MSG_HEARTBEAT             97
#define MSG_HANDSHAKE             98
#define MSG_TOGGLE_SIM            99

/* Group get messages */
#define MSG_GET_ALL_STATES            100
#define MSG_GET_ALL_TARGET_POSES      101
#define MSG_GET_ALL_VELOCITIES        102
#define MSG_GET_REGISTERED_ROBOTS     103
#define MSG_GET_AVAILABLE_ROBOTS      104

/* Group set messages */
#define MSG_SET_ALL_CURRENT_POSES     150
#define MSG_SET_ALL_TARGET_POSES      151
#define MSG_SET_ALL_VELOCITIES        152
#define MSG_STOP_ALL_ROBOTS           153

#endif
