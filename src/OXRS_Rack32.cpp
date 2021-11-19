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
OXRS_LCD _screen(Ethernet, _mqtt);

// REST API
EthernetServer _server(REST_API_PORT);
OXRS_API _api(_mqtt);

// Temp sensor
Adafruit_MCP9808 _tempSensor;

// Firmware details
const char *    _fwName;
const char *    _fwShortName;
const char *    _fwMaker;
const char *    _fwVersion;
const uint8_t * _fwLogo;
 
// Supported firmware config and command schemas
DynamicJsonDocument _fwConfigSchema(2048);
DynamicJsonDocument _fwCommandSchema(2048);

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
void _getFirmwareJson(JsonVariant json)
{
  JsonObject firmware = json.createNestedObject("firmware");

  firmware["name"] = _fwName;
  firmware["shortName"] = _fwShortName;
  firmware["maker"] = _fwMaker;
  firmware["version"] = _fwVersion;
}

void _getNetworkJson(JsonVariant json)
{
  byte mac[6];
  Ethernet.MACAddress(mac);
  
  char mac_display[18];
  sprintf_P(mac_display, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  JsonObject network = json.createNestedObject("network");

  network["ip"] = Ethernet.localIP();
  network["mac"] = mac_display;
}

void _getConfigJson(JsonVariant json)
{
  JsonObject config = json.createNestedObject("config");
  
  // Config schema metadata
  config["$schema"] = "http://json-schema.org/draft-04/schema#";
  config["description"] = _fwName;
  config["type"] = "object";

  JsonObject properties = config.createNestedObject("properties");

  // Firmware config schema (if any)
  if (!_fwConfigSchema.isNull())
  {
    _mergeJson(properties, _fwConfigSchema.as<JsonVariant>());
  }

  // Rack32 config
  JsonObject temperatureUpdateMillis = properties.createNestedObject("temperatureUpdateMillis");
  temperatureUpdateMillis["type"] = "integer";
  temperatureUpdateMillis["minimum"] = 0;
}

void _getCommandJson(JsonVariant json)
{
  JsonObject command = json.createNestedObject("command");
  
  // Command schema metadata
  command["$schema"] = "http://json-schema.org/draft-04/schema#";
  command["description"] = _fwName;
  command["type"] = "object";

  JsonObject properties = command.createNestedObject("properties");

  // Firmware command schema (if any)
  if (!_fwCommandSchema.isNull())
  {
    _mergeJson(properties, _fwCommandSchema.as<JsonVariant>());
  }

  // Rack32 commands
  JsonObject restart = properties.createNestedObject("restart");
  restart["type"] = "boolean";
}

/* MQTT callbacks */
void _mqttConnected() 
{
  // Build device adoption info
  DynamicJsonDocument json(4096);
  JsonVariant adopt = json.as<JsonVariant>();
  
  _getFirmwareJson(adopt);
  _getNetworkJson(adopt);
  _getConfigJson(adopt);
  _getCommandJson(adopt);

  // Publish device adoption info
  _mqtt.publishAdopt(adopt);

  // Log the fact we are now connected
  Serial.println("[ra32] mqtt connected");
}

void _mqttDisconnected(int state) 
{
  // Log the disconnect reason
  // See https://github.com/knolleary/pubsubclient/blob/2d228f2f862a95846c65a8518c79f48dfc8f188c/src/PubSubClient.h#L44
  switch (state)
  {
    case MQTT_CONNECTION_TIMEOUT:
      Serial.println(F("[ra32] mqtt connection timeout"));
      break;
    case MQTT_CONNECTION_LOST:
      Serial.println(F("[ra32] mqtt connection lost"));
      break;
    case MQTT_CONNECT_FAILED:
      Serial.println(F("[ra32] mqtt connect failed"));
      break;
    case MQTT_DISCONNECTED:
      Serial.println(F("[ra32] mqtt disconnected"));
      break;
    case MQTT_CONNECT_BAD_PROTOCOL:
      Serial.println(F("[ra32] mqtt bad protocol"));
      break;
    case MQTT_CONNECT_BAD_CLIENT_ID:
      Serial.println(F("[ra32] mqtt bad client id"));
      break;
    case MQTT_CONNECT_UNAVAILABLE:
      Serial.println(F("[ra32] mqtt unavailable"));
      break;
    case MQTT_CONNECT_BAD_CREDENTIALS:
      Serial.println(F("[ra32] mqtt bad credentials"));
      break;      
    case MQTT_CONNECT_UNAUTHORIZED:
      Serial.println(F("[ra32] mqtt unauthorised"));
      break;      
  }
}

void _mqttConfig(JsonVariant json)
{
  // Check for library config
  if (json.containsKey("temperatureUpdateMillis"))
  {
    _temp_update_ms = json["temperatureUpdateMillis"].as<uint32_t>();
  }
  
  // Pass on to the firmware callback
  if (_onConfig) { _onConfig(json); }
}

void _mqttCommand(JsonVariant json)
{
  // Check for library commands
  if (json.containsKey("restart") && json["restart"].as<bool>())
  {
    ESP.restart();
  }
  
  // Pass on to the firmware callback
  if (_onCommand) { _onCommand(json); }
}

void _mqttCallback(char * topic, byte * payload, int length) 
{
  // Update screen
  _screen.trigger_mqtt_rx_led();

  // Pass down to our MQTT handler and check it was processed ok
  int state = _mqtt.receive(topic, payload, length);
  switch (state)
  {
    case MQTT_RECEIVE_ZERO_LENGTH:
      Serial.println(F("[ra32] empty mqtt payload received"));
      break;
    case MQTT_RECEIVE_JSON_ERROR:
      Serial.println(F("[ra32] failed to deserialise mqtt json payload"));
      break;
    case MQTT_RECEIVE_NO_CONFIG_HANDLER:
      Serial.println(F("[ra32] no mqtt config handler"));
      break;
    case MQTT_RECEIVE_NO_COMMAND_HANDLER:
      Serial.println(F("[ra32] no mqtt command handler"));
      break;
  }
}

/* Main program */
OXRS_Rack32::OXRS_Rack32(const char * fwName, const char * fwShortName, const char * fwMaker, const char * fwVersion, const uint8_t * fwLogo)
{
  _fwName       = fwName;
  _fwShortName  = fwShortName;
  _fwMaker      = fwMaker;
  _fwVersion    = fwVersion;
  _fwLogo       = fwLogo;
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

void OXRS_Rack32::setConfigSchema(JsonVariant json)
{
  _mergeJson(_fwConfigSchema.as<JsonVariant>(), json);
}

void OXRS_Rack32::setCommandSchema(JsonVariant json)
{
  _mergeJson(_fwCommandSchema.as<JsonVariant>(), json);
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
  
  // Set up the screen
  _initialiseScreen();

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

void OXRS_Rack32::loop(void)
{
  // Check our network connection
  if (_isNetworkConnected())
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

boolean OXRS_Rack32::publishStatus(JsonVariant json)
{
  // Exit early if no network connection
  if (!_isNetworkConnected()) { return false; }

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

boolean OXRS_Rack32::publishTelemetry(JsonVariant json)
{
  // Exit early if no network connection
  if (!_isNetworkConnected()) { return false; }

  boolean success = _mqtt.publishTelemetry(json);
  if (success) { _screen.trigger_mqtt_tx_led(); }
  return success;
}

void OXRS_Rack32::_initialiseScreen(void)
{
  // Initialise the LCD
  _screen.begin();

  // Display the firmware and logo (either from SPIFFS or PROGMEM)
  int returnCode = _screen.draw_header(_fwShortName, _fwMaker, _fwVersion, "ESP32", _fwLogo);
  
  switch (returnCode)
  {
    case LCD_INFO_LOGO_FROM_SPIFFS:
      Serial.println(F("[ra32] logo loaded from SPIFFS"));
      break;
    case LCD_INFO_LOGO_FROM_PROGMEM:
      Serial.println(F("[ra32] logo loaded from PROGMEM"));
      break;
    case LCD_INFO_LOGO_DEFAULT:
      Serial.println(F("[ra32] no logo found, using default OXRS logo"));
      break;
    case LCD_ERR_NO_LOGO:
      Serial.println(F("[ra32] no logo found"));
      break;
  }
}

void OXRS_Rack32::_initialiseEthernet(byte * mac)
{
  // Get ESP32 base MAC address
  WiFi.macAddress(mac);
  
  // Ethernet MAC address is base MAC + 3
  // See https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system.html#mac-address
  mac[5] += 3;

  // Display the MAC address on serial
  char mac_display[18];
  sprintf_P(mac_display, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print(F("[ra32] mac address: "));
  Serial.println(mac_display);

  // Initialise ethernet library
  Ethernet.init(ETHERNET_CS_PIN);

  // Reset Wiznet W5500
  pinMode(WIZNET_RESET_PIN, OUTPUT);
  digitalWrite(WIZNET_RESET_PIN, HIGH);
  delay(250);
  digitalWrite(WIZNET_RESET_PIN, LOW);
  delay(50);
  digitalWrite(WIZNET_RESET_PIN, HIGH);
  delay(350);

  // Get an IP address via DHCP and display on serial
  Serial.print(F("[ra32] ip address: "));
  if (Ethernet.begin(mac, DHCP_TIMEOUT_MS, DHCP_RESPONSE_TIMEOUT_MS))
  {
    Serial.println(Ethernet.localIP());
  }
  else
  {
    Serial.println(F("none"));
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
  
  // Start listening for MQTT messages
  _mqttClient.setCallback(_mqttCallback);
}

void OXRS_Rack32::_initialiseRestApi(void)
{
  // NOTE: this must be called *after* initialising MQTT since that sets
  //       the default client id, which has lower precendence than MQTT
  //       settings stored in file and loaded by the API

  // Set up the REST API
  _api.begin();  
}

void OXRS_Rack32::_initialiseTempSensor(void)
{
  // Start the I2C bus
  Wire.begin();

  // Initialise the onboard MCP9808 temp sensor
  _tempSensor.begin(MCP9808_I2C_ADDRESS);
  _tempSensor.setResolution(MCP9808_MODE);
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
  
    StaticJsonDocument<32> json;
    json["temperature"] = payload;
    publishTelemetry(json.as<JsonVariant>());

    // Reset our timer
    _lastTempUpdate = millis();
  }
}

boolean OXRS_Rack32::_isNetworkConnected(void)
{
  // TODO: Add check for WiFi status if we add support for WiFi
  return Ethernet.linkStatus() == LinkON;
}