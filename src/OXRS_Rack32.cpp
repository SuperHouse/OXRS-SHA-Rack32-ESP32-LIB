/*
 * OXRS_Rack32.cpp
 */

#include "Arduino.h"
#include "OXRS_Rack32.h"

#include <Wire.h>                     // For I2C
#include <Ethernet.h>                 // For networking
#include <WiFi.h>                     // Required for Ethernet to get MAC
#include <Adafruit_MCP9808.h>         // For temp sensor
#include <PubSubClient.h>             // For MQTT
#include <aWOT.h>                     // For REST API
#include <SPIFFS.h>                   // For ESP32 file system

// Filename where MQTT settings are persisted on the file system
static const char * MQTT_JSON_FILENAME = "/mqtt.json";

// Ethernet client
EthernetClient _client;

// REST API
EthernetServer _server(REST_API_PORT);
Application _api;

// MQTT client
PubSubClient _mqttClient(_client);
OXRS_MQTT _mqtt(_mqttClient);

// LCD screen
OXRS_LCD _screen(Ethernet);

// Temp sensor
Adafruit_MCP9808 _tempSensor;

// Firmware details
const char * _fwName;
const char * _fwShortName;
const char * _fwMakerCode;
const char * _fwMakerName;
const char * _fwVersion;

// MQTT callbacks wrapped by _mqttConfig/_mqttCommand
jsonCallback _onConfig;
jsonCallback _onCommand;

// Temperature update interval - extend or disable temp updates via 
// the MQTT config option "temperatureUpdateMillis" - zero to disable
//
// WARNING: depending how long it takes to read the temp sensor, 
//          you might see event detection/processing interrupted
uint32_t _temp_update_ms = DEFAULT_TEMP_UPDATE_MS;

/* File system helpers */
void _mountFS()
{
  Serial.print(F("[file] mounting SPIFFS..."));
  if (!SPIFFS.begin())
  { 
    Serial.println(F("failed, might need formatting?"));
    return; 
  }
  Serial.println(F("done"));
}

boolean _formatFS()
{
  Serial.print(F("[file] formatting SPIFFS..."));
  if (!SPIFFS.format())
  { 
    Serial.println(F("failed"));
    return false; 
  }
  Serial.println(F("done"));
  return true;
}

boolean _loadJson(DynamicJsonDocument * json, const char * filename)
{
  Serial.print(F("[file] reading "));  
  Serial.print(filename);
  Serial.print(F("..."));

  File file = SPIFFS.open(filename, "r");
  if (!file) 
  {
    Serial.println(F("failed to open file"));
    return false;
  }
  
  if (file.size() == 0)
  {
    Serial.println(F("empty"));
    return false;
  }

  Serial.print(file.size());
  Serial.println(F(" bytes read"));
  
  DeserializationError error = deserializeJson(*json, file);
  if (error) 
  {
    Serial.print(F("[erro] failed to deserialise JSON: "));
    Serial.println(error.f_str());
    return false;
  }
  
  return json->isNull() ? false : true;
}

boolean _saveJson(DynamicJsonDocument * json, const char * filename)
{
  Serial.print(F("[file] writing "));
  Serial.print(filename);
  Serial.print(F("..."));

  File file = SPIFFS.open(filename, "w");
  if (!file) 
  {
    Serial.println(F("failed to open file"));
    return false;
  }

  Serial.print(serializeJson(*json, file));
  Serial.println(F(" bytes written"));
  return true;
}

boolean _deleteFile(const char * filename)
{
  Serial.print(F("[file] deleting "));
  Serial.print(filename);
  Serial.print(F("..."));

  if (!SPIFFS.remove(filename))
  {
    Serial.println(F("failed to delete file"));
    return false;
  }

  Serial.println(F("done"));
  return true;
}

/* REST API handlers */
void _getFirmwareJson(JsonObject * json)
{
  json->getOrAddMember("name").set(_fwName);
  json->getOrAddMember("shortName").set(_fwShortName);
  json->getOrAddMember("makerCode").set(_fwMakerCode);
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

void _getIndex(Request &req, Response &res)
{
  Serial.println(F("[api ] index"));

  DynamicJsonDocument json(512);
  
  JsonObject firmware = json.createNestedObject("firmware");
  _getFirmwareJson(&firmware);

  JsonObject network = json.createNestedObject("network");
  _getNetworkJson(&network);
  
  JsonObject mqtt = json.createNestedObject("mqtt");
  _mqtt.getJson(&mqtt);
  
  res.set("Content-Type", "application/json");
  serializeJson(json, req);
}

void _postReboot(Request &req, Response &res)
{
  Serial.println(F("[api ] reboot"));

  // Restart the device
  res.sendStatus(204);
  ESP.restart();
}

void _postFactoryReset(Request &req, Response &res)
{
  Serial.print(F("[api ] factory reset"));

  DynamicJsonDocument json(64);
  deserializeJson(json, req);
  
  // Factory reset - either wiping setup/config data only or format file system
  if (json.isNull() || !json["formatFileSystem"].as<boolean>())
  {
    Serial.println(F(" (delete settings)"));
    if (!_deleteFile(MQTT_JSON_FILENAME))
    {
      res.sendStatus(500);
      return;
    }
  }
  else
  {
    Serial.println(F(" (format file system)"));
    if (!_formatFS())
    {
      res.sendStatus(500);
      return;
    }
  }

  // Restart the device
  _postReboot(req, res);
}

void _postMqtt(Request &req, Response &res)
{
  Serial.println(F("[api ] update MQTT settings"));

  DynamicJsonDocument json(2048);
  deserializeJson(json, req);

  JsonObject mqtt = json.as<JsonObject>();
  _mqtt.setJson(&mqtt);
  
  _mqtt.reconnect();
  
  if (!_saveJson(&json, MQTT_JSON_FILENAME))
  {
    res.sendStatus(500);
  }
  else
  {
    res.sendStatus(204);
  }    
}

/* MQTT callbacks */
void _mqttConnected() 
{
  // Update screen
  _screen.show_mqtt_connection_status(true);
  
  // Build device discovery details
  DynamicJsonDocument json(512);
  
  JsonObject firmware = json.createNestedObject("firmware");
  _getFirmwareJson(&firmware);

  JsonObject network = json.createNestedObject("network");
  _getNetworkJson(&network);

  // Publish device discovery details (retained)
  char topic[64];

  sprintf_P(topic, PSTR("%s/%s"), _mqtt.getStatusTopic(topic), "firmware");
  _mqtt.publish(firmware, topic, true);
  
  sprintf_P(topic, PSTR("%s/%s"), _mqtt.getStatusTopic(topic), "network");
  _mqtt.publish(network, topic, true);
}

void _mqttDisconnected() 
{
  // Update screen
  _screen.show_mqtt_connection_status(false);
}

void _mqttConfig(JsonObject json)
{
  if (json.containsKey("temperatureUpdateMillis"))
  {
    _temp_update_ms = json["temperatureUpdateMillis"].as<uint32_t>();
  }
  
  if (_onConfig) { _onConfig(json); }
}

void _mqttCommand(JsonObject json)
{
  // Any Rack32 commands can be handled in here
  if (_onCommand) { _onCommand(json); }
}

void _mqttCallback(char * topic, byte * payload, int length) 
{
  // Update screen
  _screen.trigger_mqtt_rx_led();

  // Pass this message down to our MQTT handler
  _mqtt.receive(topic, payload, length);
}

/* Main program */
OXRS_Rack32::OXRS_Rack32(const char * fwName, const char * fwShortName, const char * fwMakerCode, const char * fwMakerName, const char * fwVersion)
{
  _fwName       = fwName;
  _fwShortName  = fwShortName;
  _fwMakerCode  = fwMakerCode;
  _fwMakerName  = fwMakerName;
  _fwVersion    = fwVersion;  
}

void OXRS_Rack32::setMqttBroker(const char * broker, uint16_t port)
{
  _mqtt.setBroker(broker, port);
}

void OXRS_Rack32::setMqttAuth(const char * username, const char * password)
{
  _mqtt.setAuth(username, password);
}

void OXRS_Rack32::setMqttClientId(const char * clientId)
{
  _mqtt.setClientId(clientId);
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
  // We wrap the callbacks so we can intercept messages intended for 
  // the Rack32 specifically - store here for use in the wrappers
  _onConfig = config;
  _onCommand = command;
  
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
  _screen.draw_header(_fwMakerCode, _fwMakerName, _fwShortName, _fwVersion, "ESP32");

  // Mount the file system
  _mountFS();

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
    
    // Maintain MQTT (process any messages)
    _mqtt.loop();
    
    // Handle any REST API requests
    EthernetClient client = _server.available();

    if (client.connected()) {
      _api.process(&client);
      client.stop();
    }    
  }
    
  // Maintain screen
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
  Serial.print(F("[eth ] getting MAC address from ESP32..."));
  WiFi.macAddress(mac);  // Temporarily populate Ethernet MAC with ESP32 Base MAC
  mac[5] += 3;           // Ethernet MAC is Base MAC + 3 (see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system.html#mac-address)

  char mac_display[18];
  sprintf_P(mac_display, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(mac_display);

  Ethernet.init(ETHERNET_CS_PIN);

  Serial.print("[eth ] resetting Wiznet W5500...");
  pinMode(WIZNET_RESET_PIN, OUTPUT);
  digitalWrite(WIZNET_RESET_PIN, HIGH);
  delay(250);
  digitalWrite(WIZNET_RESET_PIN, LOW);
  delay(50);
  digitalWrite(WIZNET_RESET_PIN, HIGH);
  delay(350);
  Serial.println("done");

  Serial.print(F("[eth ] getting IP address via DHCP..."));
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
  // Set the default client id to the last 3 bytes of the MAC address
  char clientId[32];
  sprintf_P(clientId, PSTR("%02x%02x%02x"), mac[3], mac[4], mac[5]);  
  _mqtt.setClientId(clientId);
  
  // Restore any persisted settings (this might overwrite the client id)
  DynamicJsonDocument json(2048);
  if (_loadJson(&json, MQTT_JSON_FILENAME))
  {
    Serial.print(F("[mqtt] restore settings from file system..."));
    JsonObject mqtt = json.as<JsonObject>();
    _mqtt.setJson(&mqtt);
    Serial.println(F("done"));
  }

  // Display the MQTT topic
  char topic[64];
  _screen.show_MQTT_topic(_mqtt.getWildcardTopic(topic));

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
  Serial.print(F("[api ] listening on :"));
  Serial.println(REST_API_PORT);

  Serial.println(F("[api ] adding / handler [get]"));
  _api.get("/", &_getIndex);
  
  Serial.println(F("[api ] adding /reboot handler [post]"));
  _api.post("/reboot", &_postReboot);

  Serial.println(F("[api ] adding /factoryReset handler [post]"));
  _api.post("/factoryReset", &_postFactoryReset);

  Serial.println(F("[api ] adding /mqtt handler [post]"));
  _api.post("/mqtt", &_postMqtt);
}

void OXRS_Rack32::_initialiseTempSensor(void)
{
  Serial.println(F("[i2c ] scanning for temperature sensor..."));
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