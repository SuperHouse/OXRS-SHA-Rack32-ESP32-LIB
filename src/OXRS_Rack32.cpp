/*
 * OXRS_Rack32.cpp
 */

#include "Arduino.h"
#include "OXRS_Rack32.h"

#include <Wire.h>                     // For I2C
#include <WiFi.h>                     // Also required for Ethernet to get MAC
#include <Ethernet.h>                 // For networking
#include <Adafruit_MCP9808.h>         // For temp sensor
#include <PubSubClient.h>             // For MQTT

// Ethernet client
EthernetClient _ethernet;

// LCD screen
OXRS_LCD _screen(Ethernet);

// MQTT client
PubSubClient _mqttClient(_ethernet);
OXRS_MQTT _mqtt(_mqttClient);

// Temp sensor
Adafruit_MCP9808 _tempSensor;

OXRS_Rack32::OXRS_Rack32(const char * fwName, const char * fwShortName, const char * fwMakerCode, const char * fwVersion, const char * fwCode)
{
  _fwName       = fwName;
  _fwShortName  = fwShortName;
  _fwMakerCode  = fwMakerCode;
  _fwVersion    = fwVersion;  
  _fwCode       = fwCode;
}

void _mqttCallback(char * topic, byte * payload, int length) 
{
  // Indicate we have received something on MQTT
  _screen.trigger_mqtt_rx_led();

  // Pass this message down to our MQTT handler
  _mqtt.receive(topic, payload, length);
}

void OXRS_Rack32::setMqttBroker(const char * broker, uint16_t port)
{
  _mqttClient.setServer(broker, port);
}

void OXRS_Rack32::setMqttAuth(const char * username, const char * password)
{
  _mqtt.setAuth(username, password);
}

void OXRS_Rack32::setMqttTopicPrefix(const char * prefix)
{
  _mqtt.setTopicPrefix(prefix);
}

void OXRS_Rack32::setMqttTopicSuffix(const char * suffix)
{
  _mqtt.setTopicSuffix(suffix);
}

void OXRS_Rack32::setDisplayPorts(uint8_t mcp23017s, int layout)
{
  _screen.draw_ports(layout, mcp23017s);
}

void OXRS_Rack32::updateDisplayPorts(uint8_t mcp, uint16_t ioValue)
{
  _screen.process(mcp, ioValue);
}

void OXRS_Rack32::begin(jsonCallback config, jsonCallback command)
{
  // Startup logging to serial
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println();
  Serial.println(F("==============================="));
  Serial.print(F(  "          OXRS by "));
  Serial.println(_fwMakerCode);
  Serial.println(_fwName);
  Serial.print  (F("             v"));
  Serial.println(_fwVersion);
  Serial.println(F("==============================="));

  // Start the I2C bus
  Wire.begin();

  // Set up the screen
  _screen.begin();

  // Display firmware details
  _screen.draw_header(_fwMakerCode, _fwShortName, _fwVersion, "ESP32");

  // Set up ethernet and obtain an IP address
  byte mac[6];
  _initialiseEthernet(mac);
  
  // Set MQTT client id and callbacks
  _mqtt.setClientId(_fwCode, mac);
  _mqtt.onConfig(config);
  _mqtt.onCommand(command);

  // Display the MQTT topic
  char topic[64];
  _screen.show_MQTT_topic(_mqtt.getWildcardTopic(topic));

  // Start listening for MQTT messages
  _mqttClient.setCallback(_mqttCallback);  

  // Set up the temperature sensor
  _initialiseTempSensor();
}

void OXRS_Rack32::loop()
{
  // Check our ethernet connection
  Ethernet.maintain();

  // Maintain MQTT (process any messages)
  _mqtt.loop();
    
  // Maintain screen
  _screen.loop();

  // Check for temperature update
  _updateTempSensor();
}

boolean OXRS_Rack32::publishStatus(JsonObject json)
{
  if (json.containsKey("index"))
  {
    char event[32];
    sprintf_P(event, PSTR("idx:%2d"), json["index"].as<uint8_t>());
    
    if (json.containsKey("type"))
    {
      sprintf_P(event, PSTR("%s %s"), event, json["type"].as<const char *>());
    }

    if (json.containsKey("event"))
    {
      sprintf_P(event, PSTR("%s %s"), event, json["event"].as<const char *>());
    }

    _screen.show_event(event);
  }
  
  boolean success = _mqtt.publishStatus(json);
  if (success) { _screen.trigger_mqtt_tx_led(); }
  return success;
}

boolean OXRS_Rack32::publishTelemetry(JsonObject json)
{
  boolean success = _mqtt.publishTelemetry(json);
  if (success) { _screen.trigger_mqtt_tx_led(); }
  return success;
}

void OXRS_Rack32::_initialiseEthernet(byte * ethernetMac)
{
  // Determine MAC address
  Serial.print(F("Getting Ethernet MAC address from ESP32: "));
  WiFi.macAddress(ethernetMac);  // Temporarily populate Ethernet MAC with ESP32 Base MAC
  ethernetMac[5] += 3;           // Ethernet MAC is Base MAC + 3 (see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system.html#mac-address)

  // Display MAC address on serial
  char mac_address[18];
  sprintf_P(mac_address, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), ethernetMac[0], ethernetMac[1], ethernetMac[2], ethernetMac[3], ethernetMac[4], ethernetMac[5]);
  Serial.println(mac_address);

  // Set up Ethernet
  Ethernet.init(ETHERNET_CS_PIN);

  // Reset the Wiznet Ethernet chip
  Serial.print("Resetting Wiznet W5500 Ethernet chip...");
  pinMode(WIZNET_RESET_PIN, OUTPUT);
  digitalWrite(WIZNET_RESET_PIN, HIGH);
  delay(250);
  digitalWrite(WIZNET_RESET_PIN, LOW);
  delay(50);
  digitalWrite(WIZNET_RESET_PIN, HIGH);
  delay(350);
  Serial.println("done");

  // Obtain IP address
  Serial.print(F("Getting IP address via DHCP: "));
  Ethernet.begin(ethernetMac);

  // Display IP address on serial
  Serial.println(Ethernet.localIP());
}

void OXRS_Rack32::_initialiseTempSensor()
{
  Serial.println(F("Scanning for temperature sensor on I2C bus..."));
  Serial.print(F(" - 0x"));
  Serial.print(MCP9808_I2C_ADDRESS, HEX);
  Serial.print(F("..."));

  if (_tempSensor.begin(MCP9808_I2C_ADDRESS))
  {
    _tempSensor.setResolution(MCP9808_MODE);

    Serial.print(F("MCP9808 (mode "));
    Serial.print(MCP9808_MODE);
    Serial.println(F(")"));
  }
  else
  {
    Serial.println(F("not found!"));
  }
}

void OXRS_Rack32::_updateTempSensor()
{
  if ((millis() - _lastTempUpdate) > MCP9808_INTERVAL_MS)
  {
    // Read temp from onboard sensor
    float temperature = _tempSensor.readTempC();
    
    // Display temp on screen
    _screen.show_temp(temperature); 

    // Publish temp to mqtt
    char payload[8];
    sprintf(payload, "%2.1f", temperature);
  
    StaticJsonDocument<64> json;
    json["temperature"] = payload;
    publishTelemetry(json.as<JsonObject>());

    // Reset our timer
    _lastTempUpdate = millis();
  }
}
