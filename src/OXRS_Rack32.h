/*
 * OXRS_Rack32.h
 */

#ifndef OXRS_RACK32_H
#define OXRS_RACK32_H

#include <OXRS_MQTT.h>                // For MQTT pub/sub
#include <OXRS_LCD.h>                 // For LCD runtime displays
#include <OXRS_API.h>                 // For REST API

// Ethernet
#define       ETHERNET_CS_PIN           26
#define       WIZNET_RESET_PIN          13
#define       DHCP_TIMEOUT_MS           15000
#define       DHCP_RESPONSE_TIMEOUT_MS  4000

// REST API
#define       REST_API_PORT             80

// Temperature update internal
#define       DEFAULT_TEMP_UPDATE_MS    60000L

// MCP9808 temperature sensor
#define       MCP9808_I2C_ADDRESS       0x18
#define       MCP9808_MODE              0
//  Mode Resolution  SampleTime
//  0    0.5°C       30 ms
//  1    0.25°C      65 ms
//  2    0.125°C     130 ms
//  3    0.0625°C    250 ms

class OXRS_Rack32 : public Print
{
  public:
    OXRS_Rack32(const uint8_t * fwLogo = NULL);

    // These are only needed if performing manual configuration in your sketch, otherwise
    // config is provisioned via the API and bootstrap page
    void setMqttBroker(const char * broker, uint16_t port);
    void setMqttClientId(const char * clientId);
    void setMqttAuth(const char * username, const char * password);
    void setMqttTopicPrefix(const char * prefix);
    void setMqttTopicSuffix(const char * suffix);

    void begin(jsonCallback config, jsonCallback command);
    void loop(void);

    // Firmware can define the config/commands it supports - for device discovery and adoption
    void setConfigSchema(JsonVariant json);
    void setCommandSchema(JsonVariant json);

    // Return a pointer to the LCD so firmware can customise if required
    // Should be called after .begin()
    OXRS_LCD* getLCD(void);

    // Helpers for standard I/O setup of the LCD
    void setDisplayPortLayout(uint8_t mcpCount, int layout);
    void setDisplayPinType(uint8_t mcp, uint8_t pin, int type);
    void setDisplayPinInvert(uint8_t mcp, uint8_t pin, int invert);
    void updateDisplayPorts(uint8_t mcp, uint16_t ioValue);
        
    // Helpers for publishing to stat/ and tele/ topics
    boolean publishStatus(JsonVariant json);
    boolean publishTelemetry(JsonVariant json);

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
    
    boolean _isNetworkConnected(void);
};

#endif
