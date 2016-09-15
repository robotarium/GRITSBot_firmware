/*
 -------------------------------------------------------------------------------
 Wifi authentication data for GRITSBot main board ESP8266 
 
 METHODS:

 NOTES: Define these variables in the corresponding cpp files to enable the 
        robots to access your WiFi network.
 
 EXAMPLES:
 
 Initially created by Daniel Pickem 6/27/15.
 -------------------------------------------------------------------------------
 */

#ifndef _WIFI_CONFIG_
#define _WIFI_CONFIG_

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
namespace wifiConfig {
	extern const char* wifiSSID;
	extern const char* wifiPassword;
	extern const char* wifiBroadcast;
}
#endif
