# vnt-can
(another) vntcontroller with canbuss display.

Goals:
See info and change settings on display/control unit over canbus network connected to sensors and vnt controller.

Sensors:
MAP
EGT
Throttle (TPS)

Boost settings:
Static boost
Map boost to TPS
MIN/MAX Boost

EGT:
Limit boost

------------------------------
boost_controller
-----------------------------

//---------------
//EEPROM
//---------------

#include <EEPROM.h>

//---------------
// CAN MCP2515
//---------------

#include <mcp_can.h>
#include <SPI.h>

//---------------
// EGT Probe
//---------------

#include "max6675.h"

//---------------
// PID
//---------------

#include <PID_v1.h>

//---------------
// MPX4250DP Pressure sensor
//---------------

#define mpx4250dp A0        // EGT Sensor Analog PIN 0

------------------------------
display_control
------------------------------

//---------------
// CAN MCP2515
//---------------

#include <mcp_can.h>
#include <SPI.h>

//---------------
// LCD I2C
//---------------

#include <LiquidCrystal_I2C.h>
#include <LCD.h>

//---------------
///Keypad
//---------------

#include <AnalogKeypad.h>
