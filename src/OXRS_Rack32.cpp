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
#include <SPIFFS.h>                   // For file system
#include <aWOT.h>                     // For REST API

// Filename where MQTT settings are persisted on the file system
static const char * MQTT_SETTING_FILENAME = "/mqtt.json";

// Ethernet client
EthernetClient _client;

// REST API
EthernetServer _server(REST_API_PORT);
Application _api;

// LCD screen
OXRS_LCD _screen(Ethernet);

// MQTT client
PubSubClient _mqttClient(_client);
OXRS_MQTT _mqtt(_mqttClient);

// Temp sensor
Adafruit_MCP9808 _tempSensor;

/* File system helpers */
void _mountFS()
{
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  Serial.print(F("[file] mounting SPIFFS..."));
  if (!SPIFFS.begin())
  { 
    Serial.println(F("failed, might need formatting?"));
    return; 
  }
  Serial.println(F("done"));
#endif
}

boolean _formatFS()
{
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  Serial.print(F("[file] formatting SPIFFS..."));
  if (!SPIFFS.format())
  { 
    Serial.println(F("failed"));
    return false; 
  }
  Serial.println(F("done"));
  return true;
#else
  return true;
#endif
}

boolean _loadJson(DynamicJsonDocument * json, const char * filename)
{
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
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
#else
  return false;
#endif
}

boolean _saveJson(DynamicJsonDocument * json, const char * filename)
{
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
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
#else
  return true;
#endif
}

boolean _deleteFile(const char * filename)
{
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
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
#endif
  return true;
}

/* REST API handlers */
void postReboot(Request &req, Response &res) 
{
  Serial.println(F("[api ] reboot"));
  
  // Restart the device
  res.sendStatus(204);
  ESP.restart();
}

void postFactoryReset(Request &req, Response &res) 
{
  Serial.print(F("[api ] factory reset"));

  DynamicJsonDocument json(64);
  deserializeJson(json, req);
  
  // Factory reset - either wiping setup/config data only or format file system
  if (json.isNull() || !json["formatFileSystem"].as<boolean>())
  {
    Serial.println(F(" (delete settings)"));
    if (!_deleteFile(MQTT_SETTING_FILENAME))
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
  postReboot(req, res);
}

void postMqtt(Request &req, Response &res) 
{
  Serial.println(F("[api ] update MQTT settings"));

  DynamicJsonDocument json(2048);
  deserializeJson(json, req);

  _mqtt.setJson(&json);
  _mqtt.reconnect();
  
  if (!_saveJson(&json, MQTT_SETTING_FILENAME))
  {
    res.sendStatus(500);
    return;
  }

  res.set("Content-Type", "application/json");
  serializeJson(json, req);
}

/* MQTT callback */
void _mqttCallback(char * topic, byte * payload, int length) 
{
  // Indicate we have received something on MQTT
  _screen.trigger_mqtt_rx_led();

  // Pass this message down to our MQTT handler
  _mqtt.receive(topic, payload, length);
}

/* Main program */
OXRS_Rack32::OXRS_Rack32(const char * fwName, const char * fwShortName, const char * fwMakerCode, const char * fwVersion, const char * fwCode)
{
  _fwName       = fwName;
  _fwShortName  = fwShortName;
  _fwMakerCode  = fwMakerCode;
  _fwVersion    = fwVersion;  
  _fwCode       = fwCode;
}

void OXRS_Rack32::setMqttBroker(const char * broker, uint16_t port)
{
  _mqtt.setBroker(broker, port);
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

  // Mount the file system
  _mountFS();

  // Set up ethernet and obtain an IP address
  _initialiseEthernet();

  // Set up MQTT
  _initialiseMqtt(config, command);

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

void OXRS_Rack32::_initialiseEthernet()
{
  Serial.print(F("[eth ] getting MAC address from ESP32..."));
  byte mac[6];
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

void OXRS_Rack32::_initialiseMqtt(jsonCallback config, jsonCallback command)
{
  // Restore any persisted settings
  DynamicJsonDocument json(2048);
  if (_loadJson(&json, MQTT_SETTING_FILENAME))
  {
    Serial.print(F("[mqtt] restore settings from file system..."));
    _mqtt.setJson(&json);
    Serial.println(F("done"));
  }

  // Display the MQTT topic
  char topic[64];
  _screen.show_MQTT_topic(_mqtt.getWildcardTopic(topic));

  // Register our callbacks
  _mqtt.onConfig(config);
  _mqtt.onCommand(command);

  // Start listening for MQTT messages
  _mqttClient.setCallback(_mqttCallback);
}

void OXRS_Rack32::_initialiseRestApi(void)
{
  _api.post("/reboot", &postReboot);
  _api.post("/factoryReset", &postFactoryReset);
  _api.post("/mqtt", &postMqtt);
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