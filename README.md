# vnt-can
(another) vntcontroller with canbuss display

------------------------------
slave / vntmaster
-----------------------------

//EEPROM

#include <EEPROM.h>

// CAN MCP2515

#include <mcp_can.h>
#include <SPI.h>

// EGT Probe

#include "max6675.h"

// PID

#include <PID_v1.h>

------------------------------
Display / controller
------------------------------

// CAN MCP2515

#include <mcp_can.h>
#include <SPI.h>

// LCD I2C

#include <LiquidCrystal_I2C.h>
#include <LCD.h>

///Keypad

#include <AnalogKeypad.h>
