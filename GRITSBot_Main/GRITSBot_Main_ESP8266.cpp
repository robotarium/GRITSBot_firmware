/*
 ------------------------------------------------------------------
 GRITSBot Main Board class

 Initially created by Daniel Pickem 7/18/14.

 Version 1.0
 ------------------------------------------------------------------
*/

//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
#include "GRITSBot_Main_ESP8266.h"

// Constructors
GRITSBotMain::GRITSBotMain(WirelessInterfaceESP8266*  radio,
                           I2CInterface*              i2c,
                           Adafruit_INA219*           ina219,
                           Adafruit_NeoPixel*         strip,
                           ControllerBase*            controller,
                           EstimatorBase*             estimator) {
  radio_      = radio;
  I2C_        = i2c;
  ina219_     = ina219;
  strip_      = strip;
  controller_ = controller;
  estimator_  = estimator;
}

// Destructor
GRITSBotMain::~GRITSBotMain() {}

//-----------------------------------------------------------------
// Public Member Functions
//-----------------------------------------------------------------
void GRITSBotMain::initialize() {
  /* Set initial time stamps */
  lastEstimatorUpdate_    = millis();
  lastControllerUpdate_   = millis();
  lastMessage_            = millis();
  lastHeartbeat_          = millis();
  lastRFMessage_          = millis();
  lastCurrentMeasurement_ = millis();
  lastI2CTest_            = millis();

  lastChargeStatusCheck_  = millis();
  lastChargeStatusCheck_  = millis();
  lastBatteryEmpytCheck_  = millis();
  lastDataTest_           = millis();
  Serial.println("Mainboard time stamps initialized");

  /* Set LED pin */
  pinMode(LED_PIN, OUTPUT);

  /* Set boost enable pin mode */
  pinMode(STEPUP_EN_PIN, OUTPUT);

  /* Set charge connector pin to input */
  pinMode(CHARGER_CONNECTION_PIN, INPUT);

  /* Set up RGB Leds using Adafruits NeoPixel library and initialize all pixels as 'off'*/
  strip_->begin();
  strip_->show();
  strip_->setBrightness(64);

  /* Initialize radio */
  radio_->initialize();
  Serial.println("Mainboard radio initialized");
  /* Initialize the message counter */
   messageCounter_ = 0;
  /* Set up mainboard as I2C master */
  I2C_->initialize(2, 14);

  /* Set I2C communication parameters */
  I2CRequestTimeout_ = 1;               /* Wait time for I2C request in ms */
  Wire.setClockStretchLimit(15000);     /* ESP8266 wait time for messages in us */
  Serial.println("Mainboard I2C initialized");

  /* Set up current sensor */
  /* NOTE: ina219_->begin() does not need to be called since
   * I2C is already set up.
   */
  ina219_->setCalibration_16V_400mA();
  Serial.println("Mainboard current monitor initialized");

  /* Get initial battery voltage and current reading */
  batteryVoltage_ = ina219_->getBusVoltage_V();
  current_.addData(ina219_->getCurrent_mA());

  /* Initialize controller */
  controller_->setL(0.05);
  controller_->setGain(20);
  Serial.println("Mainboard controller initialized");

  /* Retrieve MAC address from WiFi device */
  MACAddress_ = radio_->getMACaddress();
  Serial.print("MAC address: "); Serial.println(MACAddress_);

  /* Set initial mode */
  mode_ = MANUAL_MODE;

  /* Set firmware version */
  EEPROM.begin(512);
  setMainBoardFirmwareVersion(FIRMWARE_VERSION);
  setMainBoardHardwareVersion(HARDWARE_VERSION);

  /* Charging chip status pin is NOT connected to ESP8266 because
   * its output is not reliable whatsoever. Charge status is
   * determined by battery voltage and CHARGER_CONNECTION_PIN.
   *
   * Charging voltage connected to GPIO 13.
   * LOW  ... charger disconnected
   * HIGH ... charger connected
   */

  /* Send status message based on EEPROM sleep flag
   * NOTE: This is done in the processUDPMessage function since the   *       robot needs to receive a host IP and port first
   */
}

void GRITSBotMain::updateMeasurements() {
  /* Set rates for all measurements */
  float rateCurrent       = 10.0;
  float rateCurrentError  =  0.2;
  float rateI2C           =  0.2;
  float rateBatteryEmpty  =  0.1;
  float rateChargeStatus  =  2.0;

  /* Measure current consumption at 10 Hz */
  if( (millis() - lastCurrentMeasurement_) > 1000 / rateCurrent) {
    /* Add current consumption data */
    current_.addData(ina219_->getCurrent_mA());

    /* Update time stamp */
    lastCurrentMeasurement_ = millis();
  }

  /* Test for current measurement errors at 0.2 Hz */
  if( (millis() - lastCurrentMeasurement_) > 1000 / rateCurrentError) {
    /* Send error message to server */
    sendErrorMessage("I2C current measurement failed.");
    Serial.println("I2C current measurement failed.");
  }

  /* Test I2C communication at 0.2 Hz */
  if( (millis() - lastI2CTest_) > 1000 / rateI2C) {
    if(!testMotorBoardI2CCommunications()) {
      /* Send error message to server */
      sendErrorMessage("I2C communication with motor board failed.");
      Serial.println("I2C communication with motor board failed.");
    }

    /* Update time stamp */
    lastI2CTest_ = millis();
  }

  /* Test for empty battery at 0.1 Hz */
  if( (millis() - lastBatteryEmpytCheck_) > 1000 / rateBatteryEmpty) {
    /* Sample current sensor on main board */
    batteryVoltage_  = ina219_->getBusVoltage_V();

    /* Threshold the battery voltage */
    if(batteryVoltage_ < 3.5) {
      sendStatusMessage("battery warning", String(batteryVoltage_));
    } else if(batteryVoltage_ < 3.2) {
      sendStatusMessage("battery depleted", String(batteryVoltage_));
    } else if(batteryVoltage_ < 3.0) {
      sendStatusMessage("battery critically low", String(batteryVoltage_));
    }

    /* Update time stamp */
    lastBatteryEmpytCheck_ = millis();
  }

  /* Measure charge status at 0.5 Hz */
  if( (millis() - lastChargeStatusCheck_ ) > 1000 / rateChargeStatus) {
    /* Update charger connection status */
    chargerConnectedPrev_ = chargerConnected_;
    chargerConnected_     = digitalRead(CHARGER_CONNECTION_PIN);

    /* Transition between 3 states
     * 1. charging
     * 2. charged
     * 3. charging failed
     */
    if(chargerConnected_ != chargerConnectedPrev_) {
      /* Transition on charger connected pin detected */
      if(isCharging()) {
        /* Charger is now connected */
        sendStatusMessage("charging", String(batteryVoltage_));
      } else {
        /* Charger is now disconnected */
        if(!isCharged(true)) {
          sendStatusMessage("charging failed", String(batteryVoltage_));
        } else {
          sendStatusMessage("charged", String(batteryVoltage_));
        }
      }
    } else {
      /* If no transition is detected, only send a status update once
         charging is done */

      /* No change on charger connected pin */
      if(isCharging()) {
        if(isCharged()) {
          if( (millis() - lastChargedStatusMessage_ ) > 2000) {
            sendStatusMessage("charged", String(batteryVoltage_));

            /* Update time stamp */
            lastChargedStatusMessage_ = millis();
          }
        } else {
          /* Sleep while waiting for charging to complete */
          //enableDeepSleep(15);
        }
      } else {
        /* No charger connected. Continue operation */
      }
    }

    /* Update time stamp */
    lastChargeStatusCheck_ = millis();
  }
}

void GRITSBotMain::updateWireless() {
  yield();
  if(radio_->isConnected()) {
    /* Send heartbeat messge */
    sendHeartbeatMessage();

    /* Send RF measurement messge */
    sendRFMessage();

    /* Receive and process UDP messages */
    if(radio_->receiveMessage() > 0) {
      /* Update time stamp of last message */
      lastMessage_ = millis();
      messageCounter_ = messageCounter_ + 1;

      /* Visual output */
      toggleLed();

      /* Process message */
      yield();
      processUDPMessage();
    }
  }
}

bool GRITSBotMain::processUDPMessage() {
  yield();
  uint8_t msgType;
  String error;

  /* Parse JSON message stored in radio_.msg */
  StaticJsonBuffer<256> jsonBuffer;

  /* Parse JSON data into buffer */
  String msg = radio_->getMessage();
  JsonObject& root = jsonBuffer.parseObject(msg);

  //if(DEBUG_LEVEL > 1) {
    Serial.println(msg);
  //}

  /* Parse message type */
  if(JSONGetNumber<uint8_t>(root, String("msgType"), msgType)) {
    switch(msgType) {
      case(MSG_SET_CURRENT_POSE):
      	{
		    	if(estimator_ != NULL) {
		    		float x = 0, y = 0, theta = 0;
				    if(JSONGetNumber<float>(root, "x", x) &&
				    		JSONGetNumber<float>(root, "y", y) &&
				    		JSONGetNumber<float>(root, "theta", theta)) {
						      /* Set current pose in estimator */
						  		estimator_->setState(State(x,y,theta));
						} else {
							error = "MSG_SET_CURRENT_POSE: Failed to parse x,y, or theta: " + msg;
					  	sendErrorMessage(error);
				    }
				  }
		      break;
        }
      case(MSG_SET_TARGET_POSE):
      	{
		      if(controller_ != NULL) {
		    		float x = 0, y = 0, theta = 0;
				    if(JSONGetNumber<float>(root, "x", x) &&
				    		JSONGetNumber<float>(root, "y", y) &&
				    		JSONGetNumber<float>(root, "theta", theta)) {
						  /* Update controller variables */
							controller_->setTargetPosition(State(x,y,theta));

							/* Set operation mode */
				      mode_ = CONTROLLER_MODE;
						} else {
							error = "MSG_SET_TARGET_POSE: Failed to parse x,y, or theta: " + msg;
					  	sendErrorMessage(error);
				    }
				  }
		      break;
        }
      case(MSG_SET_VELOCITIES):
      	{
      		float v = 0, w = 0;
		      /* NOTE: This message type is used for remote control applications */
		      /* Update velocities in the controller */
		      if(JSONGetNumber<float>(root, "v", v) &&
              JSONGetNumber<float>(root, "w", w)) {
		        /* Update velocities of the main board */
		      	setVelocities(v, w);

	 	        /* Set operation mode */
			      mode_ = MANUAL_MODE;
		      } else {
		        error = "MSG_SET_VELOCITIES: Failed to parse v or w: " + msg;
				  	sendErrorMessage(error);
		      }
		      break;
        }
      case(MSG_SET_VELOCITIES_MAX):
      	{
      		float vMax = 0, wMax = 0;
		      if(JSONGetNumber<float>(root, "vMax", vMax) &&
              JSONGetNumber<float>(root, "wMax", wMax)) {
		        /* Update maximum velocities of the motor board */
						setVelocitiesMax(vMax,wMax);
		      } else {
		        error = "MSG_SET_VELOCITIES_MAX: Failed to parse vMax or wMax: " + msg;
				  	sendErrorMessage(error);
		      }
		      break;
        }
      case(MSG_SET_RPS):
	      {
	      	float rpsL = 0, rpsR = 0;
		      if(JSONGetNumber<float>(root, "rpsL", rpsL) &&
              JSONGetNumber<float>(root, "rpsR", rpsR)) {
		        /* Update RPS values of the motor board */
						setRPS(rpsL, rpsR);
		      } else {
		        error = "MSG_SET_RPS: Failed to parse rpsL or rpsR: " + msg;
				  	sendErrorMessage(error);
		      }
		      break;
        }
      case(MSG_SET_RPS_MAX):
      	{
      		float rpsMax = 0;
		      if(JSONGetNumber<float>(root, "rpsMax", rpsMax)) {
		        /* Update maximum RPS values of the motor board */
						setRPSMax(rpsMax);
		      } else {
		        error = "MSG_SET_RPS_MAX: Failed to parse rpsMax: " + msg;
				  	sendErrorMessage(error);
		      }
		      break;
        }
      case(MSG_SET_STEPS_PER_REV):
      	{
      		float steps = 0;
		      if(JSONGetNumber<float>(root, "steps", steps)) {
		        /* Update steps per revolution value for the motor
             * board's stepper motors */
						setStepsPerRevolution(steps);
		      } else {
		        error = "MSG_SET_STEPS_PER_REV: Failed to parse steps: " + msg;
				  	sendErrorMessage(error);
		      }
		      break;
        }
      case(MSG_SET_LED_RGB):
      	{
          uint8_t  index;
          uint32_t color;
          if(JSONGetNumber<uint8_t>(root, "index", index)) {
            if(JSONGetNumber<uint32_t>(root, "color", color)) {
              setLedRGB(index, color);
            }
          }
          break;
        }
      case(MSG_SET_ALL_LED_RGB):
      	{
          uint32_t color;

          if(JSONGetNumber<uint32_t>(root, "color", color)) {
            setLedsRGB(color);
          }
          break;
        }
      case (MSG_SET_RAINBOW):
        {
          uint16_t duration;
          if (JSONGetNumber<uint16_t>(root,"duration",duration)) {
            rainbow(duration);
            disableLedsRGB();
          }
        }
      case(MSG_SET_LED_RGB_EFFECT):
        {
          uint16_t reps;
          String effectType = JSONGetString(root, String("effectType"));
          if(effectType == "rainbow") {
            if(JSONGetNumber<uint16_t>(root, "reps", reps)) {
              rainbow(10, reps);
            } else {
              rainbow(10, 1);
            }
            disableLedsRGB();
          } else {

          }
        }
	  case(MSG_INTERPOLATE_COLORS):
		{
			  uint16_t duration;
		      float frequency;
			  uint8_t color1[3] = {0, 0, 0}, color2[3] = {255, 255, 255},
			  		  r1, g1,b1, r2, g2, b2;
			  if(JSONGetNumber<uint16_t>(root, "d", duration)) {
			  	  if(JSONGetNumber<float>(root, "f", frequency)) {
			  		  if(JSONGetNumber<uint8_t>(root, "R1", r1)) {
			  			  if(JSONGetNumber<uint8_t>(root, "G1", g1)) {
			  				  if(JSONGetNumber<uint8_t>(root, "B1", b1)) {
			  					  if(JSONGetNumber<uint8_t>(root, "R2", r2)) {
			  						  if(JSONGetNumber<uint8_t>(root, "G2", g2)) {
			  							  if(JSONGetNumber<uint8_t>(root, "B2", b2)) {
			  								  color1[0] = r1;
			  								  color1[1] = g1;
			  								  color1[2] = b1;
			  								  color2[0] = r2;
			  								  color2[1] = g2;
			  								  color2[2] = b2;
			  								  setGTColor(duration, frequency, color1, color2);
			  					              disableLedsRGB();
			  							  }
			  						  }
			  					  }
			  				  }
			  			  }
			  		  }
			  	  }
			  }
		  break;
		}
      case(MSG_GET_BATT_VOLT):
      	{
		      String fields[3] = {"msgType", "vBat", "iBat"};
				  float data[3]    = {MSG_GET_BATT_VOLT,
				  										batteryVoltage_,
				  										current_.getAverage()};
				  JSONSendMessage(fields, data, 3);
		      break;
        }
      case(MSG_GET_BATT_EMPTY):
      	{
		      String fields[2] = {"msgType", "battEmpty"};
				  float data[2]    = {MSG_GET_BATT_EMPTY, isBatteryEmpty()};
				  JSONSendMessage(fields, data, 2);
		      break;
        }
      case(MSG_GET_RPS_MAX):
        {
		      String fields[2] = {"msgType", "rpsMax"};
				  float data[2]    = {MSG_GET_RPS_MAX, sampleMotorBoardRPSMax()};
				  JSONSendMessage(fields, data, 2);
		      break;
        }
      case(MSG_GET_FIRMWARE_VERSION):
      	{
		      String fields[3] = {"msgType",
                              "versionMain",
                              "versionMotor"};
				  float data[3]    = {MSG_GET_FIRMWARE_VERSION,
                              getMainBoardFirmwareVersion(),
                              getMotorBoardFirmwareVersion()};
				  JSONSendMessage(fields, data, 3);
		      break;
        }
      case(MSG_GET_HARDWARE_VERSION):
      	{
		      String fields[3] = {"msgType",
                              "versionMain",
                              "versionMotor"};
				  float data[3]    = {MSG_GET_HARDWARE_VERSION,
                              getMainBoardHardwareVersion(),
                              getMotorBoardHardwareVersion()};
				  JSONSendMessage(fields, data, 3);
		      break;
        }
      case(MSG_HOST_IP):
      	{
		      /* Parse host IP */
		      int portIncoming, portOutgoing;

		      if(root.containsKey("host")) {
		        radio_->setHostIP(root["host"].as<String>());
		        sendStatusMessage("MSG_HOST_IP received");
		      } else {
		        error = "MSG_HOST_IP: Failed to parse host IP: " + msg;
		        sendErrorMessage(error);
		      }

		      /* Parse incoming port */
		      if(JSONGetNumber<int>(root, "receive_on", portIncoming)) {
		        radio_->setPortIncoming(portIncoming);
		      } else {
		        error = "MSG_HOST_IP: Failed to parse incoming port: " + msg;
		        sendErrorMessage(error);
		      }

		      /* Parse outgoing port */
		      if(JSONGetNumber<int>(root, "send_to", portOutgoing)) {
		        radio_->setPortOutgoing(portOutgoing);
		      } else {
		        error = "MSG_HOST_IP: Failed to parse outgoing port: " + msg;
		        sendErrorMessage(error);
		      }

		      /* Send status message based on EEPROM sleep flag */
		      bool sleepFlag = EEPROM.read(0);
		      if(sleepFlag) {
		        /* Send status message: wake up after sleep */
		        sendStatusMessage("woke up");

		        /* Reset sleep bit in EEPROM */
		        EEPROM.write(0, false);
		      } else {
		        /* Send status message: boot up */
		        sendStatusMessage("powered up");
		      }

		      break;
        }
      case(MSG_DEEP_SLEEP):
      	{
			    uint32_t sleepTime;

		      /* Activate deep sleep */
		      if(JSONGetNumber<uint32_t>(root, "sleepDuration", sleepTime)) {
		        enableDeepSleep(sleepTime);
		      } else {
			      error = "MSG_DEEP_SLEEP: Failed to parse 'sleepDuration': " + msg;
		        sendErrorMessage(error);
		      }
		      break;
        }
      default:
        return false;
    }
  } else {
	  error = "Message contains no message type field: " + msg;
    sendErrorMessage(error);
  }
  return true;
}

void GRITSBotMain::sendHeartbeatMessage() {
  bool printChargeStatusInfo = false;
  bool printBatteryVoltage = false;
  yield();

  /* Send heartbeat message once every second */
  if( (millis() - lastHeartbeat_) > 1000) {
    /* Charge status debugging output */
    if(printChargeStatusInfo) {
      printChargeStatus();
    }

    if(printBatteryVoltage) {
      batteryVoltage_  = ina219_->getBusVoltage_V();
      Serial.print("Battery voltage: "); Serial.println(batteryVoltage_);
    }

    if (!radio_->getHostIPStatus()) {
      /* These values signal to the host that robot has no Host IP */
      JSONSendMessage("MAC", MACAddress_);
    } else {
      /* Gather all data relevant for statistical analysis
       * 1. V_bat           ... battery voltage                   [V]
       * 2. I_bat           ... battery current                   [A]
       * 3. rps_left_avg    ... average left motor velocity     [RPS]
       * 4. rps_right_avg   ... average right motor velocity    [RPS]
       * 5. temp_left_avg   ... average left motor temperature    [C]
       * 6. temp_right_avg  ... average right motor temperature   [C]
       * 7. I_left_avg      ... average left motor current        [A]
       * 8. I_right_avg     ... average right motor current       [A]
       * 9. V_5V            ... step-up converter output voltage  [V]
       * 10. messageCounter ... number of messages received/sec   [/sec]
       */
      /* Update battery voltage and current values */
      batteryVoltage_ = ina219_->getBusVoltage_V();
      current_.addData(ina219_->getCurrent_mA());

      /* Update average RPS values */
      float rpsL = 0.0;
      float rpsR = 0.0;
      sampleMotorBoardAverageRPS(rpsL, rpsR);

      /* Update average temperatures */
      float tempL = 0.0;
      float tempR = 0.0;
      sampleMotorBoardAverageTemperatures(tempL, tempR);

      /* Update average currents */
      float curL = 0.0;
      float curR = 0.0;
      sampleMotorBoardAverageCurrents(curL, curR);

      /* Update step-up converter voltage */
      float voltageStepUp = getStepUpVoltage();

      float isChargingVal = (float) isCharging();

      /* Create heartbeat message */
      String fields[12]  = {"msgType", "vBat", "iBat", "rpsL", "rpsR", "tempL", "tempR", "iMotorL", "iMotorR", "msgRecRate", "vBoost", "charging"};
      float data[12]     = {MSG_HEARTBEAT, batteryVoltage_, current_.getAverage(),
                            rpsL, rpsR, tempL, tempR, curL, curR, (float) messageCounter_, voltageStepUp, isChargingVal};

      /* Send heartbeat message via UDP */
      JSONSendMessage(fields, data, 12);
    }

    /* Update timestamp */
    lastHeartbeat_ = millis();

    /* Reset the message counter */
    messageCounter_ = 0;

    /* Visual output */
    toggleLed();
  }
}

void GRITSBotMain::sendRFMessage() {
  /* Retrieve basic WiFi channel data
   *
   * 1. Channel number
   * 2. Physical mode (802.11 B/G/N ... 1/2/3)
   *    WIFI_PHY_MODE_11B = 1, WIFI_PHY_MODE_11G = 2, WIFI_PHY_MODE_11N = 3
   * 3. RSSI
   * 4. SSID
   *
   */
  yield();

  if( (millis() - lastRFMessage_) > 1000) {
    if (radio_->getHostIPStatus()) {
      /* Get channel number */
      uint8_t channel = wifi_get_channel();

      /* Get physical mode */
      uint8_t phyMode = wifi_get_phy_mode();

      /* Get SSID */
      struct station_config conf;
      wifi_station_get_config(&conf);
      const char* ssid = reinterpret_cast<const char*>(conf.ssid);

      /* Get RSSI */
      int32_t rssi = wifi_station_get_rssi();

      /* Create heartbeat message */
      String fields[5]  = {"msgType", "channel", "phyMode", "rssi", "ssid"};
      String data[5]    = {String(MSG_RF_DATA), String(channel), String(phyMode),
                           String(rssi), String(ssid)};

      /* Send RF message via UDP */
      JSONSendMessage(fields, data, 5);

      /* Update time stamp */
      lastRFMessage_ = millis();
    }
  }
}

/* ************************************
 *    JSON COMMUNICATION FUNCTIONS
 **************************************/
void GRITSBotMain::JSONSendMessage(String* fields, float* data, int len) {
  /* Create JSON buffer */
  yield();
  StaticJsonBuffer<400> jsonBuffer;

  /* Create JSON root object */
  JsonObject& root = jsonBuffer.createObject();

  /* Fill message with data */
  for (int i = 0; i < len; i++) {
    root[fields[i]] = data[i];
  }

  /* Print to string */
  String msg;
  root.printTo(msg);

  /* Send message via UDP */
  JSONSendMessage(msg);
}

void GRITSBotMain::JSONSendMessage(String* fields, String* data, int len) {
  /* Create JSON buffer */
  yield();
  StaticJsonBuffer<256> jsonBuffer;

  /* Create JSON root object */
  JsonObject& root = jsonBuffer.createObject();

  /* Fill message with data */
  for (int i = 0; i < len; i++) {
    root[fields[i]] = data[i];
  }

  /* Print to string */
  String msg;
  root.printTo(msg);

  /* Send message via UDP */
  JSONSendMessage(msg);
}

void GRITSBotMain::JSONSendMessage(String field, String data) {
  String fields[1]  = {field};
  String d[1]       = {data};
  JSONSendMessage(fields, d, 1);
}

void GRITSBotMain::JSONSendMessage(String field, float data) {
  String fields[1]  = {field};
  float d[1]        = {data};
  JSONSendMessage(fields, d, 1);
}

void GRITSBotMain::JSONSendMessage(String JSONData) {
  radio_->sendMessage(JSONData);
}

void GRITSBotMain::sendErrorMessage(String error, String parameters) {
  if(parameters.length() > 0) {
    /* Send message via UDP */
    String fields[3]  = {"msgType", "msg", "parameters"};
    String data[3]     = {String(MSG_ERROR), error, parameters};

    /* Send charger connection message */
    JSONSendMessage(fields, data, 3);
  } else {
    /* Send message via UDP */
    String fields[2]  = {"msgType", "msg"};
    String data[2]     = {String(MSG_ERROR), error};

    /* Send charger connection message */
    JSONSendMessage(fields, data, 2);
  }
}

void GRITSBotMain::sendStatusMessage(String status, String parameters) {
  if(parameters.length() > 0) {
    /* Create message */
    String fields[3]  = {"msgType", "msg", "parameters"};
    String data[3]     = {String(MSG_STATUS), status, parameters};

    /* Send JSON status message via UDP*/
    JSONSendMessage(fields, data, 3);
  } else {
    String fields[2]  = {"msgType", "msg"};
    String data[2]     = {String(MSG_STATUS), status};

    /* Send JSON status message via UDP*/
    JSONSendMessage(fields, data, 2);
  }

  if(DEBUG_LEVEL > 1) {
    Serial.print("Status: ");
    Serial.print(status);
    Serial.print(", ");
    Serial.println(parameters);
  }
}

/* ************************************
 *    JSON PRIMITIVES
 **************************************/
template <typename T> bool GRITSBotMain::JSONGetNumber(JsonObject& root, String field, T& output) {
  if(root.containsKey(field)) {
    if(root[field].is<T>()) {
      output = root[field].as<T>();
      return 1;
    } else {
      return 0;
    }
  } else {
    return 0;
  }
}

String GRITSBotMain::JSONGetString(JsonObject& root, String field) {
  if(root.containsKey(field)) {
    return root[field].asString();
  } else {
    return String("");
  }
}

/* **************************************
 *      CONTROLS-RELATED FUNCTIONS
 ****************************************/
void GRITSBotMain::updateController() {
  /* Compute linear and rotational velocity
   *  v ... m/s
   *  w ... rad/sec
   */
  if(mode_ == CONTROLLER_MODE) {
    if(controller_->distanceToTarget(estimator_->getState()) > 0.05) {
      /* Compute updated velocities */
      controller_->update(estimator_->getState());

      /* Update velocities */
      setVelocities(controller_->getV(), controller_->getW());
    }
  }

  /* Stop robot if no messages were received within 500 ms */
  if(millis() - lastMessage_ > 500){
    /* Update zero velocity settings at 5 Hz */
    if( (millis() - lastControllerUpdate_) > 1000 / 5) {
      setVelocities(0.0, 0.0);
      controller_->setV(0.0);
      controller_->setW(0.0);

      /* Update time stamp */
      lastControllerUpdate_ = millis();
    }
  }
}

void GRITSBotMain::updateEstimator() {
   /* NOTE: An update using overhead tracking feedback
    *       is done through the wireless update function
    */

  if( (millis() - lastEstimatorUpdate_) > 50) {
    estimator_->update(controller_->getV(), controller_->getW());
    lastEstimatorUpdate_ = millis();
  }
}

/* ************************
 *    STATUS FUNCTIONS
 **************************/
bool GRITSBotMain::isBatteryEmpty() {
  /* Sample current sensor on main board */
  batteryVoltage_  = ina219_->getBusVoltage_V();

  /* Threshold the battery voltage */
  if(batteryVoltage_ < 3.2) {
    return true;
  } else {
    return false;
  }
}

bool GRITSBotMain::isCharging() {
  /* Charge voltage VChrg pulls GPIO 13 high if connected */
  if(digitalRead(CHARGER_CONNECTION_PIN) == HIGH) {
    return true;
  } else {
    return false;
  }
}

bool GRITSBotMain::isCharged(bool disconnected) {
  if(disconnected) {
    if(getBatteryVoltage() >= 3.95) {
      return true;
    } else {
      return false;
    }
  } else {
    if(getBatteryVoltage() >= 4.1) {
      return true;
    } else {
      return false;
    }
  }
}

void GRITSBotMain::printChargeStatus() {
  batteryVoltage_  = ina219_->getBusVoltage_V();

  if(DEBUG_LEVEL > 1) {
    Serial.print("CHARGER_CONNECTION_PIN / volt: ");
    Serial.print(digitalRead(CHARGER_CONNECTION_PIN));
    Serial.print(" / ");
    Serial.println(batteryVoltage_);
  }
}

/* ***************************
 * POWER MANAGEMENT FUNCTIONS
 ****************************/
void GRITSBotMain::enableMotorVoltage() {
  digitalWrite(STEPUP_EN_PIN, HIGH);
}

void GRITSBotMain::disableMotorVoltage() {
  digitalWrite(STEPUP_EN_PIN, LOW);
}

/* ***************************
 * SLEEP FUNCTIONS
 ****************************/
void GRITSBotMain::enableDeepSleep() {
  /* Put the robot to sleep indefinitely (both main and motor board)
   *
   * NOTE: Only a power cycle will wake the robot up
   */
  /* Set sleep bit in EEPROM */
  EEPROM.write(0, true);

  /* Activate sleep */
  enableDeepSleep(0);
}

void GRITSBotMain::enableDeepSleep(uint32_t duration) {
  /* NOTE: Waking up requires the motor board to reset the main board through a
   *       short HIGH - LOW - HIGH pulse on the reset pin.
   * NOTE: This function is called in updateWireless only in the
   *       case when the battery voltage drops below 3.2 V without
   *       a charger being connected to avoid damaging the battery
   *       through under-charging.
   * NOTE: If called with a value duration > 0, the motor board will wake up
   *       the main board through a RESET after duration seconds
   * NOTE: This function reduces power consumption to <10 mA
   * NOTE: The wirewriteregister has to be made public in the Adafruit_INA219
   *       library
   */
  /* Set sleep bit in EEPROM */
  EEPROM.write(0, true);

  /* Send status message to server */
  sendStatusMessage("sleeping", String(duration));
  delay(500);

  /* Enable motor board deep sleep */
  I2C_->sendMessage(MSG_DEEP_SLEEP, duration, 0.0);
  delay(10);

  /* Power down current sensor INA219 */
  /* NOTE: The sensor won't have to be woken up, since the board will be reset
   * instead of woken up. As such, the sensor will be reinitialized on reset.
   * See documentation of Adafruit_INA219 library and example on
   *   https://github.com/jarzebski/Arduino-INA219/blob/master/INA219_simple/INA219_simple.ino
   */
  enableCurrentSensorSleep();

  /* Power off LED */
  ledOff();

  /* Power down ESP8266 chip on main board */
  ESP.deepSleep(duration);
}

void GRITSBotMain::enableCurrentSensorSleep() {
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
    INA219_CONFIG_GAIN_1_40MV |
    INA219_CONFIG_BADCRES_12BIT |
    INA219_CONFIG_SADCRES_12BIT_1S_532US |
    INA219_CONFIG_MODE_POWERDOWN;
  ina219_->wireWriteRegister(INA219_REG_CONFIG, config);
}

void GRITSBotMain::disableCurrentSensorSleep() {
  ina219_->setCalibration_16V_400mA();
}

/* *************************
 *      SET FUNCTIONS
 ***************************/
void GRITSBotMain::setVelocities(float v, float w) {
  /* NOTE: v is in [m/sec]
   *       w is in [rad/sec]
   *
   * NOTE: Motor board expects [deg/sec].
   */
  v_.fval = v;
  w_.fval = w;
  I2C_->sendMessage(MSG_SET_VELOCITIES, v, w * 180 / M_PI);
}

void GRITSBotMain::setVelocitiesMax(float vMax, float wMax) {
  /* NOTE: vMax is in [m/sec]
   *       wMax is in [rad/sec]
   *
   * NOTE: Motor board expects [deg/sec]
   */
  I2C_->sendMessage(MSG_SET_VELOCITIES_MAX, vMax, wMax * 180/M_PI);
}

void GRITSBotMain::setRPS(float rpsLeft, float rpsRight) {
  I2C_->sendMessage(MSG_SET_RPS, rpsLeft, rpsRight);
}

void GRITSBotMain::setRPSMax(float rpsMax) {
  I2C_->sendMessage(MSG_SET_RPS_MAX, rpsMax, 0.0);
}

void GRITSBotMain::setStepsPerRevolution(float steps) {
  I2C_->sendMessage(MSG_SET_STEPS_PER_REV, steps, 0.0);
}

/* *************************
 *      GET FUNCTIONS
 ***************************/
State GRITSBotMain::getCurrentPosition() {
  return estimator_->getState();
}

State GRITSBotMain::getTargetPosition() {
  return controller_->getTargetPosition();
}

float GRITSBotMain::getBatteryVoltage() {
  batteryVoltage_ = ina219_->getBusVoltage_V();
  return batteryVoltage_;
}

float GRITSBotMain::getCurrentConsumption() {
  return ina219_->getCurrent_mA();
}

float GRITSBotMain::getStepUpVoltage() {
  /* NOTE: The ADC input of the ADC returns a value 0 - 1023,
   *       which corresponds to 0 - 1 V
   * NOTE: A resistor divider maps the input voltage of 0 - 6 V
   *       to a range of 0 - 1 V, which the ESP8266 ADC expects.
   */
  uint16_t voltage = analogRead(A0);
  return map(voltage, 0, 1023, 0, 6);
}

/* ***********************************
 *  GET DATA FROM MOTORBOARD FUNCTIONS
 *************************************/
bool GRITSBotMain::sampleMotorBoardVelocities() {
  I2C_->sendMessage(MSG_GET_VELOCITIES, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  /* NOTE: Motor board expects [deg/sec].
   */
  if(I2C_->receiveMessage(&I2CBuffer_)) {
    v_.fval = I2CBuffer_.data_[0].fval;
    w_.fval = I2CBuffer_.data_[1].fval * M_PI / 180;
    return true;
  } else {
    return false;
  }
}

bool GRITSBotMain::sampleMotorBoardRPS() {
  I2C_->sendMessage(MSG_GET_RPS, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    rpsLeft_.fval = I2CBuffer_.data_[0].fval;
    rpsRight_.fval = I2CBuffer_.data_[1].fval;
    return true;
  }

  return false;
}

bool GRITSBotMain::sampleMotorBoardRPSMax() {
  I2C_->sendMessage(MSG_GET_RPS_MAX, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    rpsMax_.fval = I2CBuffer_.data_[0].fval;
    return true;
  }

  return false;
}

bool GRITSBotMain::sampleMotorBoardAverageRPS(float& rpsL, float& rpsR) {
  I2C_->sendMessage(MSG_GET_AVG_RPS, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    rpsL = I2CBuffer_.data_[0].fval;
    rpsR = I2CBuffer_.data_[1].fval;
    return true;
  }

  return false;
}

bool GRITSBotMain::sampleMotorBoardAverageTemperatures(float& tempL, float& tempR) {
  I2C_->sendMessage(MSG_GET_AVG_TEMPERATURES, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    tempL = I2CBuffer_.data_[0].fval;
    tempR = I2CBuffer_.data_[1].fval;
    return true;
  }

  return false;
}

bool GRITSBotMain::sampleMotorBoardAverageCurrents(float& curL, float& curR) {
  I2C_->sendMessage(MSG_GET_AVG_CURRENTS, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    curL = I2CBuffer_.data_[0].fval;
    curR = I2CBuffer_.data_[1].fval;
    return true;
  }

  return false;
}

bool GRITSBotMain::testMotorBoardI2CCommunications() {
  /* This function checks if I2C communication between motor and main
   * board works correctly by sending 2 float values and checking their return
   * values for a match
   */
  float r1 = 19;
  float r2 = 83;
  I2C_->sendMessage(MSG_ECHO, r1, r2);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    /* Check if the echoed data matches the sent data */
    if(I2CBuffer_.data_[0].fval == r1 && I2CBuffer_.data_[1].fval == r2) {
      return true;
    } else {
      return false;
    }
  }
  return false;
}

/* *************************
 *      LED FUNCTIONS
 ***************************/
void GRITSBotMain::toggleLed() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void GRITSBotMain::ledOn() {
  digitalWrite(LED_PIN, HIGH);
}

void GRITSBotMain::ledOff() {
  digitalWrite(LED_PIN, LOW);
}

/* *************************
 *      RGB LED FUNCTIONS
 ***************************/
void GRITSBotMain::setLedRGB(uint8_t index, uint32_t color) {
  if(index >= 0 && index < strip_->numPixels()) {
    strip_->setPixelColor(index, color);
    strip_->show();
  }
}

void GRITSBotMain::setLedsRGB(uint32_t color) {
  for(uint16_t i = 0; i < strip_->numPixels(); i++) {
    strip_->setPixelColor(i, color);
  }
  strip_->show();
}

void GRITSBotMain::disableLedRGB(uint8_t index) {
  if(index >= 0 && index < strip_->numPixels()) {
    strip_->setPixelColor(index, 0);
    strip_->show();
  }
}

void GRITSBotMain::disableLedsRGB() {
  for(uint16_t i = 0; i < strip_->numPixels(); i++) {
    strip_->setPixelColor(i, 0);
  }

  strip_->show();
}

/* Taken from Adafruit's NeoPixel library */
void GRITSBotMain::rainbow(uint8_t wait, uint8_t repetitions) {
  uint16_t i, j, r;

  for(r = 0; r < repetitions; r++) {
    for(j = 0; j < 256; j++) {
      for(i = 0; i < strip_->numPixels(); i++) {
        strip_->setPixelColor(i, Wheel((i+j) & 255));
      }

      strip_->show();
      delay(wait);
    }
  }
}

void GRITSBotMain::setGTColor(uint16_t duration, float frequency, uint8_t color1[3], uint8_t color2[3]) {
	uint32_t color[3] = {0, 0, 0};
	uint32_t strip_color;
	uint32_t start_time = millis();
	uint32_t t = 0;
	float lambda = 0;
	while ( t < duration*1000 ) {
		t = millis()-start_time;
		lambda = 0.5+0.5*sin(2*3.14*frequency*float(t)/1000);
		color[0] = uint32_t(color1[0] + lambda * (color2[0]-color1[0]));
		color[1] = uint32_t(color1[1] + lambda * (color2[1]-color1[1]));
		color[2] = uint32_t(color1[2] + lambda * (color2[2]-color1[2]));
		strip_color = strip_->Color(color[0], color[1], color[2]);
		setLedsRGB(strip_color);
		delay(10);
	}
}

/* Taken from Adafruit's NeoPixel library
 * Input a value 0 to 255 to get a color value.
 * The colours are a transition r - g - b - back to r.
 */
uint32_t GRITSBotMain::Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip_->Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip_->Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip_->Color(WheelPos * 3, 255 - WheelPos * 3,
        0);
  }
}

/* *************************
 *    UTILITY FUNCTIONS
 ***************************/
float GRITSBotMain::map(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/* *************************
 *    VERSIONING FUNCTIONS
 ***************************/
bool GRITSBotMain::setMainBoardFirmwareVersion(uint32_t version) {
  uint8_t i = EEPROM_writeAnything(FIRMWARE_ADDRESS, version);

  if(i > 0) {
    return true;
  } else {
    return false;
  }
}

bool GRITSBotMain::setMainBoardHardwareVersion(uint32_t version) {
  uint8_t i = EEPROM_writeAnything(HARDWARE_ADDRESS, version);

  if(i > 0) {
    return true;
  } else {
    return false;
  }
}

uint32_t GRITSBotMain::getMainBoardFirmwareVersion() {
  uint32_t version;
  uint8_t i = EEPROM_readAnything(FIRMWARE_ADDRESS, version);

  if(i > 0) {
    return version;
  } else {
    return false;
  }
}

uint32_t GRITSBotMain::getMainBoardHardwareVersion() {
  uint32_t version;
  uint8_t i = EEPROM_readAnything(HARDWARE_ADDRESS, version);

  if(i > 0) {
    return version;
  } else {
    return false;
  }
}

uint32_t GRITSBotMain::getMotorBoardFirmwareVersion() {
  I2C_->sendMessage(MSG_GET_FIRMWARE_VERSION, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    return (uint32_t) I2CBuffer_.data_[0].fval;
  }

  return false;
}

uint32_t GRITSBotMain::getMotorBoardHardwareVersion() {
  I2C_->sendMessage(MSG_GET_HARDWARE_VERSION, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    return (uint32_t) I2CBuffer_.data_[0].fval;
  }

  return false;
}
