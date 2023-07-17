/*
 * OXRS_Rack32.h
 */

#ifndef OXRS_RACK32_H
#define OXRS_RACK32_H

#include <OXRS_MQTT.h>                // For MQTT pub/sub
#include <OXRS_API.h>                 // For REST API
#include <OXRS_LCD.h>                 // For LCD runtime displays

// WifiManager
#define       WM_CONFIG_PORTAL_TIMEOUT_S  300

// Ethernet
#define       ETHERNET_CS_PIN             26
#define       WIZNET_RESET_PIN            13
#define       DHCP_TIMEOUT_MS             15000
#define       DHCP_RESPONSE_TIMEOUT_MS    4000

// I2C
#define       I2C_SDA                     21
#define       I2C_SCL                     22

// REST API
#define       REST_API_PORT               80

// Temperature update internal
#define       DEFAULT_TEMP_UPDATE_MS      60000L

// MCP9808 temperature sensor
#define       MCP9808_I2C_ADDRESS         0x18
#define       MCP9808_MODE                0
//  Mode Resolution  SampleTime
//  0    0.5°C       30 ms
//  1    0.25°C      65 ms
//  2    0.125°C     130 ms
//  3    0.0625°C    250 ms

class OXRS_Rack32 : public Print
{
  public:
    OXRS_Rack32(const uint8_t * fwLogo = NULL);

    void begin(jsonCallback config, jsonCallback command);
    void loop(void);

    // Firmware can define the config/commands it supports - for device discovery and adoption
    void setConfigSchema(JsonVariant json);
    void setCommandSchema(JsonVariant json);

    // Return a pointer to the MQTT library
    OXRS_MQTT * getMQTT(void);

    // Return a pointer to the API library
    OXRS_API * getAPI(void);

    // Return a pointer to the LCD so firmware can customise if required
    // Should be called after .begin()
    OXRS_LCD * getLCD(void);
    
    // Helpers for publishing to stat/ and tele/ topics
    bool publishStatus(JsonVariant json);
    bool publishTelemetry(JsonVariant json);

    // Implement Print.h wrapper
    virtual size_t write(uint8_t);
    using Print::write;

  private:
    void _initialiseScreen(void);
    void _initialiseNetwork(byte * mac);
    void _initialiseMqtt(byte * mac);
    void _initialiseRestApi(void);
    void _initialiseTempSensor(void);

    void _updateTempSensor(void);
    uint32_t _lastTempUpdate;
    
    bool _isNetworkConnected(void);
};

#endif
