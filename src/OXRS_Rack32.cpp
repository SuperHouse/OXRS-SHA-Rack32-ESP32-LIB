/*
 * OXRS_Rack32.cpp
 */

#include "Arduino.h"
#include "OXRS_Rack32.h"
#include "bootstrap_html.h"

#include <Wire.h>                     // For I2C
#include <Ethernet.h>                 // For networking
#include <WiFi.h>                     // Required for Ethernet to get MAC
#include <Adafruit_MCP9808.h>         // For temp sensor
#include <PubSubClient.h>             // For MQTT

// Ethernet client
EthernetClient _client;

// MQTT client
PubSubClient _mqttClient(_client);
OXRS_MQTT _mqtt(_mqttClient);

// LCD screen
OXRS_LCD _screen(Ethernet);

// REST API
EthernetServer _server(REST_API_PORT);
OXRS_API _api(_mqtt);

// Temp sensor
Adafruit_MCP9808 _tempSensor;

// Firmware details
const char * _fwName;
const char * _fwShortName;
const char * _fwMaker;
const char * _fwVersion;

// Supported device config
DynamicJsonDocument _deviceConfig(4096);

// MQTT callbacks wrapped by _mqttConfig/_mqttCommand
jsonCallback _onConfig;
jsonCallback _onCommand;

// Temperature update interval - extend or disable temp updates via 
// the MQTT config option "temperatureUpdateMillis" - zero to disable
//
// WARNING: depending how long it takes to read the temp sensor, 
//          you might see event detection/processing interrupted
uint32_t _temp_update_ms = DEFAULT_TEMP_UPDATE_MS;

/* JSON helpers */
void _mergeJson(JsonVariant dst, JsonVariantConst src)
{
  if (src.is<JsonObject>())
  {
    for (auto kvp : src.as<JsonObjectConst>())
    {
      _mergeJson(dst.getOrAddMember(kvp.key()), kvp.value());
    }
  }
  else
  {
    dst.set(src);
  }
}

/* MQTT adoption info builders */
void _getFirmwareJson(JsonObject * json)
{
  json->getOrAddMember("name").set(_fwName);
  json->getOrAddMember("shortName").set(_fwShortName);
  json->getOrAddMember("maker").set(_fwMaker);
  json->getOrAddMember("version").set(_fwVersion);
}

void _getNetworkJson(JsonObject * json)
{
  byte mac[6];
  Ethernet.MACAddress(mac);
  
  char mac_display[18];
  sprintf_P(mac_display, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  json->getOrAddMember("ip").set(Ethernet.localIP());
  json->getOrAddMember("mac").set(mac_display);
}

void _getDeviceConfigJson(JsonObject * json)
{
  // Add Rack32 specific config
  JsonObject temperatureUpdateMillis = json->createNestedObject("temperatureUpdateMillis");
  
  temperatureUpdateMillis.getOrAddMember("type").set("long");
  temperatureUpdateMillis.getOrAddMember("description").set("Set temperature update interval, zero to disable updates");

  // Merge any device config from the firmware
  for (auto kvp : _deviceConfig.as<JsonObject>())
  {
    _mergeJson(json->getOrAddMember(kvp.key()), kvp.value());
  }
}

/* MQTT callbacks */
void _mqttConnected() 
{
  char topic[64];

  // Update screen
  _screen.show_MQTT_topic(_mqtt.getWildcardTopic(topic));
  _screen.show_mqtt_connection_status(true);
  
  // Build device adoption info
  DynamicJsonDocument json(4096);
  
  JsonObject firmware = json.createNestedObject("firmware");
  _getFirmwareJson(&firmware);

  JsonObject network = json.createNestedObject("network");
  _getNetworkJson(&network);

  JsonObject deviceConfig = json.createNestedObject("deviceConfig");
  _getDeviceConfigJson(&deviceConfig);

  // Publish device adoption info
  _mqtt.publishAdopt(json.as<JsonObject>());
}

void _mqttDisconnected() 
{
  // Update screen
  _screen.show_MQTT_topic("");
  _screen.show_mqtt_connection_status(false);
}

void _mqttConfig(JsonObject json)
{
  if (json.containsKey("temperatureUpdateMillis"))
  {
    _temp_update_ms = json["temperatureUpdateMillis"].as<uint32_t>();
  }
  
  // Pass on to our parents callback
  if (_onConfig) { _onConfig(json); }
}

void _mqttCommand(JsonObject json)
{
  // Pass on to our parents callback
  if (_onCommand) { _onCommand(json); }
}

void _mqttCallback(char * topic, byte * payload, int length) 
{
  // Update screen
  _screen.trigger_mqtt_rx_led();

  // Pass down to our MQTT handler
  _mqtt.receive(topic, payload, length);
}

/* Main program */
OXRS_Rack32::OXRS_Rack32(const char * fwName, const char * fwShortName, const char * fwMaker, const char * fwVersion)
{
  _fwName       = fwName;
  _fwShortName  = fwShortName;
  _fwMaker      = fwMaker;
  _fwVersion    = fwVersion;  
}

void OXRS_Rack32::setMqttBroker(const char * broker, uint16_t port)
{
  _mqtt.setBroker(broker, port);
}

void OXRS_Rack32::setMqttClientId(const char * clientId)
{
  _mqtt.setClientId(clientId);
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

void OXRS_Rack32::setDeviceConfig(JsonObject json)
{
  _mergeJson(_deviceConfig.as<JsonVariant>(), json);
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
  // We wrap the callbacks so we can intercept messages intended for the Rack32
  _onConfig = config;
  _onCommand = command;
  
  // Startup logging to serial
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println();
  Serial.println(F("==============================="));
  Serial.println(_fwName);
  Serial.print  (F("             v"));
  Serial.println(_fwVersion);
  Serial.println(F("==============================="));

  // Start the I2C bus
  Wire.begin();

  // Set up the screen
  _screen.begin();

  // Display firmware details
  _screen.draw_header(_fwShortName, _fwMaker, _fwVersion, "ESP32");

  // Set up ethernet and obtain an IP address
  byte mac[6];
  _initialiseEthernet(mac);

  // Set up MQTT (don't attempt to connect yet)
  _initialiseMqtt(mac);

  // Set up the REST API
  _initialiseRestApi();

  // Set up the temperature sensor
  _initialiseTempSensor();
}

void OXRS_Rack32::loop()
{
  // Check our ethernet connection
  if (Ethernet.linkStatus() == LinkON)
  {
    // Maintain our DHCP lease
    Ethernet.maintain();
    
    // Handle any MQTT messages
    _mqtt.loop();
    
    // Handle any REST API requests
    EthernetClient client = _server.available();
    _api.checkEthernet(&client);
  }
    
  // Update screen
  _screen.loop();

  // Check for temperature update
  _updateTempSensor();
}

boolean OXRS_Rack32::publishStatus(JsonObject json)
{
  // Check for something we can show on the screen
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

void OXRS_Rack32::_initialiseEthernet(byte * mac)
{
  Serial.print(F("[ra32] getting MAC address from ESP32..."));
  WiFi.macAddress(mac);  // Temporarily populate Ethernet MAC with ESP32 Base MAC
  mac[5] += 3;           // Ethernet MAC is Base MAC + 3 (see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system.html#mac-address)

  char mac_display[18];
  sprintf_P(mac_display, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(mac_display);

  Ethernet.init(ETHERNET_CS_PIN);

  Serial.print("[ra32] resetting Wiznet W5500...");
  pinMode(WIZNET_RESET_PIN, OUTPUT);
  digitalWrite(WIZNET_RESET_PIN, HIGH);
  delay(250);
  digitalWrite(WIZNET_RESET_PIN, LOW);
  delay(50);
  digitalWrite(WIZNET_RESET_PIN, HIGH);
  delay(350);
  Serial.println("done");

  Serial.print(F("[ra32] getting IP address via DHCP..."));
  if (Ethernet.begin(mac, DHCP_TIMEOUT_MS, DHCP_RESPONSE_TIMEOUT_MS))
  {
    Serial.println(Ethernet.localIP());
  }
  else
  {
    Serial.println(F("failed"));
  }
}

void OXRS_Rack32::_initialiseMqtt(byte * mac)
{
  // NOTE: this must be called *before* initialising the REST API since
  //       that will load MQTT config from file, which has precendence

  // Set the default client ID to last 3 bytes of the MAC address
  char clientId[32];
  sprintf_P(clientId, PSTR("%02x%02x%02x"), mac[3], mac[4], mac[5]);  
  _mqtt.setClientId(clientId);
  
  // Register our callbacks
  _mqtt.onConnected(_mqttConnected);
  _mqtt.onDisconnected(_mqttDisconnected);
  _mqtt.onConfig(_mqttConfig);
  _mqtt.onCommand(_mqttCommand);

  // Set the max buffer size so we can handle large messages
  _mqttClient.setBufferSize(MQTT_MAX_MESSAGE_SIZE);
  
  // Start listening for MQTT messages
  _mqttClient.setCallback(_mqttCallback);
}

void OXRS_Rack32::_initialiseRestApi(void)
{
  // NOTE: this must be called *after* initialising MQTT since that sets
  //       the default client id, which has lower precendence than MQTT
  //       settings stored in file and loaded by the API

  Serial.print(F("[ra32] starting api on :"));
  Serial.println(REST_API_PORT);

  // Set up the REST API
  _api.begin();  
}

void OXRS_Rack32::_initialiseTempSensor(void)
{
  Serial.println(F("[ra32] scanning for temperature sensor..."));
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

void OXRS_Rack32::_updateTempSensor(void)
{
  if (_temp_update_ms > 0 && (millis() - _lastTempUpdate) > _temp_update_ms)
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