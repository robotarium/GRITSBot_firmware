/*
 -------------------------------------------------------------------------------
 Wireless Interface Class for the ESP8266 WiFi Module

 METHODS:

 NOTES:

 EXAMPLES:

 Initially created by Daniel Pickem 6/15/15.
 -------------------------------------------------------------------------------
 */

#ifndef _WIRELESS_INTERFACE_ESP8266_h_
#define _WIRELESS_INTERFACE_ESP8266_h_

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
/* Include ESP8266 headers for Wifi and UDP */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <IPAddress.h>

/* WiFi authentication data */
#include "GRITSBot_WiFiConfig/wifiConfig.h"

/* Include message definitions */
#include "GRITSBot_Messages/GRITSBot_Messages.h"

/* Include low-level ESP8266 functions */
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

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define BUFFER_SIZE     256

class WirelessInterfaceESP8266 {
  //--------------------------------------------------------------------------
  // Public Member Functions
  //--------------------------------------------------------------------------
  public:
    /* Constructors */
    WirelessInterfaceESP8266();

    /* Destructor */
    ~WirelessInterfaceESP8266();

    /* Setup function */
    bool initialize();
    bool switchAP(String SSID, String password);

    /* Wireless communication functions */
    int32_t  receiveMessage();
    void    sendMessage(String data);

    /* Status functions */
    bool isConnected();

    /* Set functions */
    void setPortIncoming(uint16_t port);
    void setPortOutgoing(uint16_t port);
    void setHostIP(String hostIP);
    bool setWifiChannel(uint8_t channel);
    void setServerSSID(String ssid);
    void setServerPassword(String password);
    void reconnectToMainHost();

    /* Get functions */
    uint16_t  getPortIncoming() { return portIncoming_; };
    uint16_t  getPortOutgoing() { return portOutgoing_; };

    IPAddress getHostIP() { return hostIP_; };
    String    getHostIPString() { return hostIPStr_; };
    IPAddress getLocalIP() { return WiFi.localIP(); };
    int       getWiFiStatus() { return WiFi.status(); };
    bool      getHostIPStatus() { return receivedHostIP_; };

    String    getMACaddress();
    String    getMessage() { return msg_; }

  //--------------------------------------------------------------------------
  // Private Member Functions
  //--------------------------------------------------------------------------
  private:
    /* Functions handling UDP package translation */
    bool      sendUdpPacket(String msg);
    int32_t    receiveUdpPacket();

    void      processHostIP(String IP_msg);
    IPAddress parseIPString(String ip);
    void      processWirelessParameters(String hostIP,
                                        int incomingPort,
                                        int outgoingPort);

  //--------------------------------------------------------------------------
  // Public Member Variables
  //--------------------------------------------------------------------------
  public:
    WiFiClient* wifiClient_;
    WiFiUDP udp_;

  //--------------------------------------------------------------------------
  // Private Member Variables
  //--------------------------------------------------------------------------
  private:
    IPAddress hostIP_;
    String hostIPStr_;
    uint16_t portIncoming_;
    uint16_t portOutgoing_;
    String serverSSID_;
    String serverPassword_;
    uint8_t wifiChannel_;
    char receiveBuffer_[BUFFER_SIZE];
    char sendBuffer_[BUFFER_SIZE];

    bool receivedHostIP_;

    String msg_;
};
#endif
