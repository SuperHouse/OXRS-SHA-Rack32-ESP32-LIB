/*
 * OXRS_Rack32.cpp
 */

#include "Arduino.h"
#include "OXRS_Rack32.h"

#include <Wire.h>                     // For I2C
#include <Ethernet.h>                 // For networking
#include <WiFi.h>                     // Required for Ethernet to get MAC
#include <MqttLogger.h>               // For logging
#include <Adafruit_MCP9808.h>         // For temp sensor

#if defined(WIFI_MODE)
#include <WiFiManager.h>              // For WiFi AP config
#endif

// Macro for converting env vars to strings
#define STRINGIFY(s) STRINGIFY1(s)
#define STRINGIFY1(s) #s

// Network client (for MQTT)/server (for REST API)
#if defined(WIFI_MODE)
WiFiClient _client;
WiFiServer _server(REST_API_PORT);
#else
EthernetClient _client;
EthernetServer _server(REST_API_PORT);
#endif

// MQTT client
PubSubClient _mqttClient(_client);
OXRS_MQTT _mqtt(_mqttClient);

// REST API
OXRS_API _api(_mqtt);

// LCD screen
#if defined(WIFI_MODE)
OXRS_LCD _screen(WiFi, _mqtt);
#else
OXRS_LCD _screen(Ethernet, _mqtt);
#endif

// Logging (topic updated once MQTT connects successfully)
MqttLogger _logger(_mqttClient, "log", MqttLoggerMode::MqttAndSerial);

// Temp sensor
Adafruit_MCP9808 _tempSensor;

// Firmware logo
const uint8_t * _fwLogo;
 
// Supported firmware config and command schemas
DynamicJsonDocument _fwConfigSchema(JSON_CONFIG_MAX_SIZE);
DynamicJsonDocument _fwCommandSchema(JSON_COMMAND_MAX_SIZE);

// MQTT callbacks wrapped by _mqttConfig/_mqttCommand
jsonCallback _onConfig;
jsonCallback _onCommand;

// Temperature update interval - extend or disable temp updates via 
// the MQTT config option "temperatureUpdateSeconds" - zero to disable
//
// WARNING: depending how long it takes to read the temp sensor, 
//          you might see event detection/processing interrupted
bool     _tempSensorFound = false;
uint32_t _tempUpdateMs    = DEFAULT_TEMP_UPDATE_MS;

/* JSON helpers */
void _mergeJson(JsonVariant dst, JsonVariantConst src)
{
  if (src.is<JsonObjectConst>())
  {
    for (JsonPairConst kvp : src.as<JsonObjectConst>())
    {
      if (dst[kvp.key()])
      {
        _mergeJson(dst[kvp.key()], kvp.value());
      }
      else
      {
        dst[kvp.key()] = kvp.value();
      }
    }
  }
  else
  {
    dst.set(src);
  }
}

/* Adoption info builders */
void _getFirmwareJson(JsonVariant json)
{
  JsonObject firmware = json.createNestedObject("firmware");

  firmware["name"] = FW_NAME;
  firmware["shortName"] = FW_SHORT_NAME;
  firmware["maker"] = FW_MAKER;
  firmware["version"] = STRINGIFY(FW_VERSION);
  
#if defined(FW_GITHUB_URL)
  firmware["githubUrl"] = FW_GITHUB_URL;
#endif
}

void _getSystemJson(JsonVariant json)
{
  JsonObject system = json.createNestedObject("system");

  system["heapUsedBytes"] = ESP.getHeapSize();
  system["heapFreeBytes"] = ESP.getFreeHeap();
  system["heapMaxAllocBytes"] = ESP.getMaxAllocHeap();
  system["flashChipSizeBytes"] = ESP.getFlashChipSize();

  system["sketchSpaceUsedBytes"] = ESP.getSketchSize();
  system["sketchSpaceTotalBytes"] = ESP.getFreeSketchSpace();

  system["fileSystemUsedBytes"] = SPIFFS.usedBytes();
  system["fileSystemTotalBytes"] = SPIFFS.totalBytes();
}

void _getNetworkJson(JsonVariant json)
{
  JsonObject network = json.createNestedObject("network");

  byte mac[6];

#if defined(WIFI_MODE)
  WiFi.macAddress(mac);

  network["mode"] = "wifi";
  network["ip"] = WiFi.localIP();
#else
  Ethernet.MACAddress(mac);

  network["mode"] = "ethernet";
  network["ip"] = Ethernet.localIP();
#endif

  char mac_display[18];
  sprintf_P(mac_display, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  network["mac"] = mac_display;
}

void _getConfigSchemaJson(JsonVariant json)
{
  JsonObject configSchema = json.createNestedObject("configSchema");
  
  // Config schema metadata
  configSchema["$schema"] = JSON_SCHEMA_VERSION;
  configSchema["title"] = FW_SHORT_NAME;
  configSchema["type"] = "object";

  JsonObject properties = configSchema.createNestedObject("properties");

  // Firmware config schema (if any)
  if (!_fwConfigSchema.isNull())
  {
    _mergeJson(properties, _fwConfigSchema.as<JsonVariant>());
  }

  // MCP9808 temp sensor config
  if (_tempSensorFound)
  {
    JsonObject temperatureUpdateSeconds = properties.createNestedObject("temperatureUpdateSeconds");
    temperatureUpdateSeconds["title"] = "Temperature Update Interval (seconds)";
    temperatureUpdateSeconds["description"] = "How often to read and report the value from the onboard MCP9808 temperature sensor (defaults to 60 seconds, setting to 0 disables temperature reports). Must be a number between 0 and 86400 (i.e. 1 day).";
    temperatureUpdateSeconds["type"] = "integer";
    temperatureUpdateSeconds["minimum"] = 0;
    temperatureUpdateSeconds["maximum"] = 86400;
  }

  // LCD config
  JsonObject activeBrightnessPercent = properties.createNestedObject("activeBrightnessPercent");
  activeBrightnessPercent["title"] = "LCD Active Brightness (%)";
  activeBrightnessPercent["description"] = "Brightness of the LCD when active (defaults to 100%). Must be a number between 0 and 100.";
  activeBrightnessPercent["type"] = "integer";
  activeBrightnessPercent["minimum"] = 0;
  activeBrightnessPercent["maximum"] = 100;
 
  JsonObject inactiveBrightnessPercent = properties.createNestedObject("inactiveBrightnessPercent");
  inactiveBrightnessPercent["title"] = "LCD Inactive Brightness (%)";
  inactiveBrightnessPercent["description"] = "Brightness of the LCD when in-active (defaults to 10%). Must be a number between 0 and 100.";
  inactiveBrightnessPercent["type"] = "integer";
  inactiveBrightnessPercent["minimum"] = 0;
  inactiveBrightnessPercent["maximum"] = 100;

  JsonObject activeDisplaySeconds = properties.createNestedObject("activeDisplaySeconds");
  activeDisplaySeconds["title"] = "LCD Active Display Timeout (seconds)";
  activeDisplaySeconds["description"] = "How long the LCD remains 'active' after an event is detected (defaults to 10 seconds, setting to 0 disables the timeout). Must be a number between 0 and 600 (i.e. 10 minutes).";
  activeDisplaySeconds["type"] = "integer";
  activeDisplaySeconds["minimum"] = 0;
  activeDisplaySeconds["maximum"] = 600;

  JsonObject eventDisplaySeconds = properties.createNestedObject("eventDisplaySeconds");
  eventDisplaySeconds["title"] = "LCD Event Display Timeout (seconds)";
  eventDisplaySeconds["description"] = "How long the last event is displayed on the LCD (defaults to 3 seconds, setting to 0 disables the timeout). Must be a number between 0 and 600 (i.e. 10 minutes).";
  eventDisplaySeconds["type"] = "integer";
  eventDisplaySeconds["minimum"] = 0;
  eventDisplaySeconds["maximum"] = 600;
}

void _getCommandSchemaJson(JsonVariant json)
{
  JsonObject commandSchema = json.createNestedObject("commandSchema");
  
  // Command schema metadata
  commandSchema["$schema"] = JSON_SCHEMA_VERSION;
  commandSchema["title"] = FW_SHORT_NAME;
  commandSchema["type"] = "object";

  JsonObject properties = commandSchema.createNestedObject("properties");

  // Firmware command schema (if any)
  if (!_fwCommandSchema.isNull())
  {
    _mergeJson(properties, _fwCommandSchema.as<JsonVariant>());
  }

  // Rack32 commands
  JsonObject restart = properties.createNestedObject("restart");
  restart["title"] = "Restart";
  restart["type"] = "boolean";
}

/* API callbacks */
void _apiAdopt(JsonVariant json)
{
  // Build device adoption info
  _getFirmwareJson(json);
  _getSystemJson(json);
  _getNetworkJson(json);
  _getConfigSchemaJson(json);
  _getCommandSchemaJson(json);
}

/* MQTT callbacks */
void _mqttConnected() 
{
  // MqttLogger doesn't copy the logging topic to an internal
  // buffer so we have to use a static array here
  static char logTopic[64];
  _logger.setTopic(_mqtt.getLogTopic(logTopic));

  // Publish device adoption info
  DynamicJsonDocument json(JSON_ADOPT_MAX_SIZE);
  _mqtt.publishAdopt(_api.getAdopt(json.as<JsonVariant>()));

  // Log the fact we are now connected
  _logger.println("[ra32] mqtt connected");
}

void _mqttDisconnected(int state) 
{
  // Log the disconnect reason
  // See https://github.com/knolleary/pubsubclient/blob/2d228f2f862a95846c65a8518c79f48dfc8f188c/src/PubSubClient.h#L44
  switch (state)
  {
    case MQTT_CONNECTION_TIMEOUT:
      _logger.println(F("[ra32] mqtt connection timeout"));
      break;
    case MQTT_CONNECTION_LOST:
      _logger.println(F("[ra32] mqtt connection lost"));
      break;
    case MQTT_CONNECT_FAILED:
      _logger.println(F("[ra32] mqtt connect failed"));
      break;
    case MQTT_DISCONNECTED:
      _logger.println(F("[ra32] mqtt disconnected"));
      break;
    case MQTT_CONNECT_BAD_PROTOCOL:
      _logger.println(F("[ra32] mqtt bad protocol"));
      break;
    case MQTT_CONNECT_BAD_CLIENT_ID:
      _logger.println(F("[ra32] mqtt bad client id"));
      break;
    case MQTT_CONNECT_UNAVAILABLE:
      _logger.println(F("[ra32] mqtt unavailable"));
      break;
    case MQTT_CONNECT_BAD_CREDENTIALS:
      _logger.println(F("[ra32] mqtt bad credentials"));
      break;      
    case MQTT_CONNECT_UNAUTHORIZED:
      _logger.println(F("[ra32] mqtt unauthorised"));
      break;      
  }
}

void _mqttConfig(JsonVariant json)
{
  // MCP9808 temp sensor config
  if (json.containsKey("temperatureUpdateSeconds"))
  {
    _tempUpdateMs = json["temperatureUpdateSeconds"].as<uint32_t>() * 1000L;
    if (_tempUpdateMs == 0L)
    {
      // wipes recent temp display on screen
      _screen.hideTemp();
    }
  }
  
  // LCD config
  if (json.containsKey("activeBrightnessPercent"))
  {
    _screen.setBrightnessOn(json["activeBrightnessPercent"].as<int>());
  }

  if (json.containsKey("inactiveBrightnessPercent"))
  {
    _screen.setBrightnessDim(json["inactiveBrightnessPercent"].as<int>());
  }

  if (json.containsKey("activeDisplaySeconds"))
  {
    _screen.setOnTimeDisplay(json["activeDisplaySeconds"].as<int>());
  }
  
  if (json.containsKey("eventDisplaySeconds"))
  {
    _screen.setOnTimeEvent(json["eventDisplaySeconds"].as<int>());
  }

  // Pass on to the firmware callback
  if (_onConfig) { _onConfig(json); }
}

void _mqttCommand(JsonVariant json)
{
  // Check for Rack32 commands
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
  _screen.triggerMqttRxLed();

  // Pass down to our MQTT handler and check it was processed ok
  int state = _mqtt.receive(topic, payload, length);
  switch (state)
  {
    case MQTT_RECEIVE_ZERO_LENGTH:
      _logger.println(F("[ra32] empty mqtt payload received"));
      break;
    case MQTT_RECEIVE_JSON_ERROR:
      _logger.println(F("[ra32] failed to deserialise mqtt json payload"));
      break;
    case MQTT_RECEIVE_NO_CONFIG_HANDLER:
      _logger.println(F("[ra32] no mqtt config handler"));
      break;
    case MQTT_RECEIVE_NO_COMMAND_HANDLER:
      _logger.println(F("[ra32] no mqtt command handler"));
      break;
  }
}

/* Main program */
OXRS_Rack32::OXRS_Rack32(const uint8_t * fwLogo)
{
  _fwLogo = fwLogo;
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

void OXRS_Rack32::begin(jsonCallback config, jsonCallback command)
{
  // Get our firmware details
  DynamicJsonDocument json(512);
  _getFirmwareJson(json.as<JsonVariant>());

  // Log firmware details
  _logger.print(F("[ra32] "));
  serializeJson(json, _logger);
  _logger.println();

  // We wrap the callbacks so we can intercept messages intended for the Rack32
  _onConfig = config;
  _onCommand = command;
  
  // Set up the screen
  _initialiseScreen();

  // Set up network and obtain an IP address
  byte mac[6];
  _initialiseNetwork(mac);

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
#if not defined(WIFI_MODE)
    Ethernet.maintain();
#endif
    
    // Handle any MQTT messages
    _mqtt.loop();
    
    // Handle any REST API requests
#if defined(WIFI_MODE)
    WiFiClient client = _server.available();
    _api.loop(&client);
#else
    EthernetClient client = _server.available();
    _api.loop(&client);
#endif
  }
    
  // Update screen
  _screen.loop();

  // Check for temperature update
  _updateTempSensor();
}

void OXRS_Rack32::setConfigSchema(JsonVariant json)
{
  _fwConfigSchema.clear();
  _mergeJson(_fwConfigSchema.as<JsonVariant>(), json);
}

void OXRS_Rack32::setCommandSchema(JsonVariant json)
{
  _fwCommandSchema.clear();
  _mergeJson(_fwCommandSchema.as<JsonVariant>(), json);
}

OXRS_LCD* OXRS_Rack32::getLCD()
{
  return &_screen;
}

void OXRS_Rack32::setDisplayPortLayout(uint8_t mcpCount, int layout)
{
  _screen.drawPorts(layout, mcpCount);
}

void OXRS_Rack32::setDisplayPinType(uint8_t mcp, uint8_t pin, int type)
{
  _screen.setPinType(mcp, pin, type);
}

void OXRS_Rack32::setDisplayPinInvert(uint8_t mcp, uint8_t pin, int invert)
{
  _screen.setPinInvert(mcp, pin, invert);
}

void OXRS_Rack32::setDisplayPinDisabled(uint8_t mcp, uint8_t pin, int disabled)
{
  _screen.setPinDisabled(mcp, pin, disabled);
}

void OXRS_Rack32::updateDisplayPorts(uint8_t mcp, uint16_t ioValue)
{
  _screen.process(mcp, ioValue);
}

void OXRS_Rack32::apiGet(const char * path, Router::Middleware * middleware)
{
  _api.get(path, middleware);
}

void OXRS_Rack32::apiPost(const char * path, Router::Middleware * middleware)
{
  _api.post(path, middleware);
}

boolean OXRS_Rack32::publishStatus(JsonVariant json)
{
  // Check for something we can show on the screen
  if (json.containsKey("index"))
  {
    // Pad the index to 3 chars - to ensure a consistent display for all indices
    char event[32];
    sprintf_P(event, PSTR("[%3d]"), json["index"].as<uint8_t>());

    if (json.containsKey("type") && json.containsKey("event"))
    {
      if (strcmp(json["type"], json["event"]) == 0)
      {
        sprintf_P(event, PSTR("%s %s"), event, json["type"].as<const char *>());
      }
      else
      {
        sprintf_P(event, PSTR("%s %s %s"), event, json["type"].as<const char *>(), json["event"].as<const char *>());
      }
    }
    else if (json.containsKey("type"))
    {
      sprintf_P(event, PSTR("%s %s"), event, json["type"].as<const char *>());
    }
    else if (json.containsKey("event"))
    {
      sprintf_P(event, PSTR("%s %s"), event, json["event"].as<const char *>());
    }

    _screen.showEvent(event);
  }
  
  // Exit early if no network connection
  if (!_isNetworkConnected()) { return false; }

  boolean success = _mqtt.publishStatus(json);
  if (success) { _screen.triggerMqttTxLed(); }
  return success;
}

boolean OXRS_Rack32::publishTelemetry(JsonVariant json)
{
  // Exit early if no network connection
  if (!_isNetworkConnected()) { return false; }

  boolean success = _mqtt.publishTelemetry(json);
  if (success) { _screen.triggerMqttTxLed(); }
  return success;
}

size_t OXRS_Rack32::write(uint8_t character)
{
  // Pass to logger - allows firmware to use `rack32.println("Log this!")`
  return _logger.write(character);
}

void OXRS_Rack32::_initialiseScreen(void)
{
  // Initialise the LCD
  _screen.begin();

  // Display the firmware and logo (either from SPIFFS or PROGMEM)
  int returnCode = _screen.drawHeader(FW_SHORT_NAME, FW_MAKER, STRINGIFY(FW_VERSION), "ESP32", _fwLogo);
  
  switch (returnCode)
  {
    case LCD_INFO_LOGO_FROM_SPIFFS:
      _logger.println(F("[ra32] logo loaded from SPIFFS"));
      break;
    case LCD_INFO_LOGO_FROM_PROGMEM:
      _logger.println(F("[ra32] logo loaded from PROGMEM"));
      break;
    case LCD_INFO_LOGO_DEFAULT:
      _logger.println(F("[ra32] no logo found, using default OXRS logo"));
      break;
    case LCD_ERR_NO_LOGO:
      _logger.println(F("[ra32] no logo found"));
      break;
  }
}

void OXRS_Rack32::_initialiseNetwork(byte * mac)
{
  // Get WiFi base MAC address
  WiFi.macAddress(mac);

#if not defined(WIFI_MODE)
  // Ethernet MAC address is base MAC + 3
  // See https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system.html#mac-address
  mac[5] += 3;
#endif

  // Format the MAC address for logging
  char mac_display[18];
  sprintf_P(mac_display, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

#if defined(WIFI_MODE)
  _logger.print(F("[ra32] wifi mac address: "));
  _logger.println(mac_display);

  // Ensure we are in the correct WiFi mode
  WiFi.mode(WIFI_STA);

  // Connect using saved creds, or start captive portal if none found
  // NOTE: Blocks until connected or the portal is closed
  WiFiManager wm;
  bool success = wm.autoConnect("OXRS_WiFi", "superhouse");

  _logger.print(F("[ra32] ip address: "));
  _logger.println(success ? WiFi.localIP() : IPAddress(0, 0, 0, 0));
#else
  _logger.print(F("[ra32] ethernet mac address: "));
  _logger.println(mac_display);

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

  // Connect ethernet and get an IP address via DHCP
  bool success = Ethernet.begin(mac, DHCP_TIMEOUT_MS, DHCP_RESPONSE_TIMEOUT_MS);
  
  _logger.print(F("[ra32] ip address: "));
  _logger.println(success ? Ethernet.localIP() : IPAddress(0, 0, 0, 0));
#endif
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
  
  // Register our callbacks
  _api.onAdopt(_apiAdopt);

  // Start listening
  _server.begin();
}

void OXRS_Rack32::_initialiseTempSensor(void)
{
  // Start the I2C bus
  Wire.begin();

  // Initialise the onboard MCP9808 temp sensor
  _tempSensorFound = _tempSensor.begin(MCP9808_I2C_ADDRESS);
  if (!_tempSensorFound)
  {
    _logger.print(F("[ra32] no MCP9808 temp sensor found at 0x"));
    _logger.println(MCP9808_I2C_ADDRESS, HEX);
    return;
  }
  
  // Set the temp sensor resolution (higher res takes longer for reading)
  _tempSensor.setResolution(MCP9808_MODE);
  _lastTempUpdate = -_tempUpdateMs;
}

void OXRS_Rack32::_updateTempSensor(void)
{
  // Ignore if temp sensor not found or has been disabled
  if (!_tempSensorFound || _tempUpdateMs == 0) { return; }
  
  // Check if we need to get a new temp reading and publish
  if ((millis() - _lastTempUpdate) > _tempUpdateMs)
  {
    // Read temp from onboard sensor
    float temperature = _tempSensor.readTempC();
    if (!isnan(temperature))
    {
      // Display temp on screen
      _screen.showTemp(temperature); 

      // Publish temp to mqtt
      StaticJsonDocument<32> json;
      json["temperature"] = temperature;
      publishTelemetry(json.as<JsonVariant>());
    }
    
    // Reset our timer
    _lastTempUpdate = millis();
  }
}

boolean OXRS_Rack32::_isNetworkConnected(void)
{
#if defined(WIFI_MODE)
  return WiFi.status() == WL_CONNECTED;
#else
  return Ethernet.linkStatus() == LinkON;
#endif
}