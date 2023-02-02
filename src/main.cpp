/**
  ESP32 security monitor firmware for the Open eXtensible Rack System

  Documentation:  
    https://oxrs.io/docs/firmware/security-monitor-esp32.html

  GitHub repository:
    https://github.com/austinscreations/OXRS-AC-SecurityMonitor-ESP32-FW

  Copyright 2022 Austins Creations
*/

/*--------------------------- Libraries -------------------------------*/
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>        // For MCP23017 I/O buffers
#include <OXRS_Input.h>               // For input handling

#if defined(OXRS_RACK32)
#include <OXRS_Rack32.h>              // Rack32 support
#include "logo.h"                     // Embedded maker logo
OXRS_Rack32 oxrs(FW_LOGO);
#elif defined(OXRS_ROOM8266)
#include <OXRS_Room8266.h>            // Room8266 support
OXRS_Room8266 oxrs;
#endif

/*--------------------------- Constants -------------------------------*/
// Serial
#define       SERIAL_BAUD_RATE      115200

// Can have up to 8x MCP23017s on a single I2C bus
const byte    MCP_I2C_ADDRESS[]     = { 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27 };
const uint8_t MCP_COUNT             = sizeof(MCP_I2C_ADDRESS);

// Each MCP23017 has 16 I/O pins, or 4 RJ45 ports (i.e. 4 pins per port)
#define       MCP_PIN_COUNT         16
#define       MCP_PORT_COUNT        4

// Set false for breakout boards with external pull-ups
#define       MCP_INTERNAL_PULLUPS  true

// Speed up the I2C bus to get faster event handling
#define       I2C_CLOCK_SPEED       400000L

/*--------------------------- Global Variables ------------------------*/
// Each bit corresponds to an MCP found on the IC2 bus
uint8_t g_mcps_found = 0;

/*--------------------------- Instantiate Globals ---------------------*/
// I/O buffers
Adafruit_MCP23X17 mcp23017[MCP_COUNT];

// Input handlers
OXRS_Input oxrsInput[MCP_COUNT];

/*--------------------------- Program ---------------------------------*/
uint8_t getMaxPort()
{
  // Count how many MCPs were found
  uint8_t mcpCount = 0;
  for (uint8_t mcp = 0; mcp < MCP_COUNT; mcp++)
  {
    if (bitRead(g_mcps_found, mcp) != 0) { mcpCount++; }
  }

  // Remember our indexes are 1-based
  return mcpCount * MCP_PORT_COUNT;  
}

void getEventType(char eventType[], uint8_t state)
{
  // Determine what event we need to publish
  sprintf_P(eventType, PSTR("error"));
  switch (state)
  {
    case HIGH_EVENT:
      sprintf_P(eventType, PSTR("normal"));
      break;
    case LOW_EVENT:
      sprintf_P(eventType, PSTR("alarm"));
      break;
    case TAMPER_EVENT:
      sprintf_P(eventType, PSTR("tamper"));
      break;
    case SHORT_EVENT:
      sprintf_P(eventType, PSTR("short"));
      break;
    case FAULT_EVENT:
      sprintf_P(eventType, PSTR("fault"));
      break;
  }
}

void setPortInvert(uint8_t port, int invert)
{
  // Work out the MCP we are configuring
  int mcp = (port - 1) / MCP_PORT_COUNT;

  // Work out the pins we need to configure for each port
  int fromPin = (port - 1) * (MCP_PIN_COUNT / MCP_PORT_COUNT);

  for (uint8_t pin = fromPin; pin < fromPin + 4; pin++)
  {
    // Configure the display
    #if defined(OXRS_RACK32)
    oxrs.setDisplayPinInvert(mcp, pin, invert);
    #endif

    // Pass this update to the input handler
    oxrsInput[mcp].setInvert(pin, invert);
  }
}

void setPortDisabled(uint8_t port, int disabled)
{
  // Work out the MCP we are configuring
  int mcp = (port - 1) / MCP_PORT_COUNT;

  // Work out the pins we need to configure for each port
  int fromPin = (port - 1) * (MCP_PIN_COUNT / MCP_PORT_COUNT);

  for (uint8_t pin = fromPin; pin < fromPin + 4; pin++)
  {
    // Configure the display
    #if defined(OXRS_RACK32)
    oxrs.setDisplayPinDisabled(mcp, pin, disabled);
    #endif

    // Pass this update to the input handler
    oxrsInput[mcp].setDisabled(pin, disabled);
  }
}

void setDisplay()
{
  // Display ports based on what MCPs were detected
  #if defined(OXRS_RACK32)
  oxrs.setDisplayPortLayout(g_mcps_found, PORT_LAYOUT_INPUT_AUTO);
  #endif

  // Setup every port as type SECURITY
  for (uint8_t mcp = 0; mcp < MCP_COUNT; mcp++)
  {
    if (bitRead(g_mcps_found, mcp) == 0)
      continue;

    for (uint8_t pin = 0; pin < MCP_PIN_COUNT; pin++)
    {
      #if defined(OXRS_RACK32)
      oxrs.setDisplayPinType(mcp, pin, PIN_TYPE_SECURITY);
      #endif
    }
  }
}

/**
  Config handler
 */
void setConfigSchema()
{
  // Define our config schema
  StaticJsonDocument<1024> json;

  JsonObject ports = json.createNestedObject("ports");
  ports["title"] = "Port Configuration";
  ports["description"] = "Add configuration for each port in use on your device. The 1-based index specifies which port you wish to configure. Inverting a port swaps the 'active' state. Disabling a port stops any events being emitted.";
  ports["type"] = "array";
  
  JsonObject items = ports.createNestedObject("items");
  items["type"] = "object";

  JsonObject properties = items.createNestedObject("properties");

  JsonObject port = properties.createNestedObject("port");
  port["title"] = "Port";
  port["type"] = "integer";
  port["minimum"] = 1;
  port["maximum"] = getMaxPort();

  JsonObject invert = properties.createNestedObject("invert");
  invert["title"] = "Invert";
  invert["type"] = "boolean";

  JsonObject disabled = properties.createNestedObject("disabled");
  disabled["title"] = "Disabled";
  disabled["type"] = "boolean";

  JsonArray required = items.createNestedArray("required");
  required.add("port");

  // Pass our config schema down to the hardware library
  oxrs.setConfigSchema(json.as<JsonVariant>());
}

uint8_t getPort(JsonVariant json)
{
  if (!json.containsKey("port"))
  {
    oxrs.println(F("[secm] missing port"));
    return 0;
  }
  
  uint8_t port = json["port"].as<uint8_t>();

  // Check the port is valid for this device
  if (port <= 0 || port > getMaxPort())
  {
    oxrs.println(F("[secm] invalid port"));
    return 0;
  }

  return port;
}

void jsonPortConfig(JsonVariant json)
{
  uint8_t port = getPort(json);
  if (port == 0) return;

  if (json.containsKey("invert"))
  {
    setPortInvert(port, json["invert"].as<bool>());
  }

  if (json.containsKey("disabled"))
  {
    setPortDisabled(port, json["disabled"].as<bool>());
  }
}

void jsonConfig(JsonVariant json)
{
  if (json.containsKey("ports"))
  {
    for (JsonVariant port : json["ports"].as<JsonArray>())
    {
      jsonPortConfig(port);    
    }
  }
}

void publishEvent(uint8_t port, uint8_t state)
{
  char eventType[7];
  getEventType(eventType, state);

  StaticJsonDocument<128> json;
  json["port"] = port;
  json["type"] = "security";
  json["event"] = eventType;

  if (!oxrs.publishStatus(json.as<JsonVariant>()))
  {
    oxrs.print(F("[secm] [failover] "));
    serializeJson(json, oxrs);
    oxrs.println();

    // TODO: add failover handling code here
  }
}

/**
  Event handlers
*/
void inputEvent(uint8_t id, uint8_t input, uint8_t type, uint8_t state)
{
  // Determine the port (1-based) for this input event
  // NOTE: the event is always generated on the last input (of 4) for each port
  uint8_t mcp = id;
  uint8_t mcpPort = (input + 1) / (MCP_PIN_COUNT / MCP_PORT_COUNT);
  uint8_t port = (MCP_PORT_COUNT * mcp) + mcpPort;

  // Publish the event
  publishEvent(port, state);
}

/**
  I2C
*/
void scanI2CBus()
{
  oxrs.println(F("[secm] scanning for I/O buffers..."));

  for (uint8_t mcp = 0; mcp < MCP_COUNT; mcp++)
  {
    oxrs.print(F(" - 0x"));
    oxrs.print(MCP_I2C_ADDRESS[mcp], HEX);
    oxrs.print(F("..."));

    // Check if there is anything responding on this address
    Wire.beginTransmission(MCP_I2C_ADDRESS[mcp]);
    if (Wire.endTransmission() == 0)
    {
      bitWrite(g_mcps_found, mcp, 1);
      
      // If an MCP23017 was found then initialise and configure the inputs
      mcp23017[mcp].begin_I2C(MCP_I2C_ADDRESS[mcp]);
      for (uint8_t pin = 0; pin < MCP_PIN_COUNT; pin++)
      {
        mcp23017[mcp].pinMode(pin, MCP_INTERNAL_PULLUPS ? INPUT_PULLUP : INPUT);
      }

      // Initialise input handlers as type SECURITY (locked to this type)
      oxrsInput[mcp].begin(inputEvent, SECURITY);

      oxrs.print(F("MCP23017"));
      if (MCP_INTERNAL_PULLUPS) { oxrs.print(F(" (internal pullups)")); }
      oxrs.println();
    }
    else
    {
      oxrs.println(F("empty"));
    }
  }
}

/**
  Setup
*/
void setup()
{
  // Start serial and let settle
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);
  Serial.println(F("[secm] starting up..."));

  // Start the I2C bus
  Wire.begin();

  // Scan the I2C bus and set up I/O buffers
  scanI2CBus();

  // Start hardware
  oxrs.begin(jsonConfig, NULL);

  // Set up display for all security ports
  setDisplay();

  // Set up config schema (for self-discovery and adoption)
  setConfigSchema();
  
  // Speed up I2C clock for faster scan rate (after bus scan)
  Wire.setClock(I2C_CLOCK_SPEED);
}

/**
  Main processing loop
*/
void loop()
{
  // Let hardware handle any events etc
  oxrs.loop();

  // Iterate through each of the MCP23017s
  for (uint8_t mcp = 0; mcp < MCP_COUNT; mcp++)
  {
    if (bitRead(g_mcps_found, mcp) == 0)
      continue;

    // Read the values for all 16 pins on this MCP
    uint16_t io_value = mcp23017[mcp].readGPIOAB();

    // Show port animations
    #if defined(OXRS_RACK32)
    oxrs.updateDisplayPorts(mcp, io_value);
    #endif

    // Check for any input events
    oxrsInput[mcp].process(mcp, io_value);
  }
}
