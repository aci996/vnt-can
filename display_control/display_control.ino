//------------------------------------------------------------------------------------------------
// Magnus Karlberg
//
//
//
//
//------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------
// FIX
// Controll relay
// Serial debug
// menu items
// set variables
//------------------------------------------------------------------------------------------------


//************************************************************************************************
// START
//************************************************************************************************

// ************************************************
// Include
// ************************************************

// CAN MCP2515
#include <mcp_can.h>
#include <SPI.h>

// LCD Kina I2C
#include <LiquidCrystal_I2C.h>
#include <LCD.h>

///Keypad
#include <AnalogKeypad.h>

// ************************************************
// Pin definitions
// ************************************************

// Canbuss
#define CAN0_INT 2    // Set INT to pin 2 (UNO)
MCP_CAN CAN0(10);     // Set CS to pin 10 (UNO)
// si		orange	ardunio icmpport	(pin 11 UNO)
// so		brun	ardunio icmpport	(pin 12 UNO)
// sck		gul		ardunio icmpport	(pin 13 UNO)

// Kinalcd
// spi A4
//     A5

// Keypad
const uint8_t KeypadAnalogPin = A3;

// Relay
// #define relay1 3

// ************************************************
// Constants / Global variables
// ************************************************

//Keypad
const uint16_t KeypadHoldTimeMs = 5000;
int button = 0;

// Define button click (button variable)
#define keypad_left   0x001
#define keypad_up     0x002
#define keypad_down   0x003
#define keypad_right  0x004
#define keypad_select 0x005

// SerialCMD
String S_inData;
bool echo = 0;
bool S_debug = 0;
const int Serial_Log_Interval = 250;
unsigned long Last_Serial_Log_Time = 0;

// Timers
const int Log_Interval = 500;
unsigned long Last_Log_Time = 0;

const int T1_Interval = 250;
unsigned long Last_T1_Time = 0;

const int LCD_Interval = 100;
unsigned long Last_LCD_Time = 0;


// Canbus
long unsigned int canbuss_rxId;
unsigned char canbuss_len = 0;
unsigned char canbuss_in[8];
unsigned char canbuss_out[8];
// char msgString[128];                        // Array to store serial string


//MAP
int mapsensor = 0;
int mapsensor_raw = 0;

//EGT
int egtsensor = 0;

//RPM
int rpmsensor = 0;

//TPS
int tps = 0;
int tps_raw = 0;

//Boostcontrol
int boost_wanted = 0;

//Cutoff
bool cutoff_act = 0;


// ************************************************
// Init
// ************************************************

// LCD
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

// Keypad
AnalogKeypad keypad(KeypadAnalogPin, KeypadHoldTimeMs);

// ************************************************
// States (menu)
// ************************************************

// Menu items
enum operatingState { OFF = 0, RUN, RAW};
operatingState opState = RUN;

// ************************************************************************************************
// Setup
// ************************************************************************************************
void setup() {

  // ************************************************
  // Pinmodes
  // ************************************************

  // Canbuss
  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

  // Relay
  // pinMode(relay1, OUTPUT);

  // ************************************************
  // Setup cont.
  // ************************************************

  // Start serial
  Serial.begin(115200);

  // LCD
  lcd.begin(20, 4); // Lcd 20x4 rows
  lcd.setCursor(0, 0);
  lcd.print(F("Controller Booting"));

  // Canbuss
  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  lcd.setCursor(0, 2);
  lcd.print("MCP2515 Init");
  Serial.print(F("MCP2515 Init"));
  lcd.setCursor(0, 3);
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    lcd.print(F("Successfully!"));
    Serial.print(F("Successfully!"));
  }
  else {
    lcd.print(F("Error..."));
    Serial.print(F("Error..."));
  }
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted.
  // Set operation mode to normal so the MCP2515 sends acks to received data.

  // Delay before loop
  delay(500);

}

// ************************************************************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************************************************************
void loop() {
  // Clear lcd every menu change
  lcd.clear();

  switch (opState)
  {
    case OFF:
      Off();
      break;
    case RUN:
      Run();
      break;
    case RAW:
      Raw();
      break;
  }
}

// ************************************************************************************************
// Functions
//
//
// ************************************************************************************************

// ************************************************
// State OFF (default)
// ************************************************
void Off()
{
  // Run once in menu
  button = 0;
  lcd.setCursor(0, 0);
  lcd.print(F("   Controller: OFF"));

  // Relay
  // digitalWrite(relay1, HIGH); // Invert LOW = ON

  while (true)
  {
    // Loop in menu
    keypad.loop(ButtonHandler); // Check keypad press
    if (button == keypad_select)
    {
      opState = RUN;
      return;
    }
    // OFF
    // DoControl();
  }
}

// ************************************************
// State RUN (default)
// ************************************************
void Run()
{
  // Run once in menu
  button = 0;
  lcd.setCursor(18, 0);
  lcd.print(F("ON"));

  // Relay
  // digitalWrite(relay1, LOW); // Invert LOW = ON

  // Display TPS
  lcd.setCursor(0, 0);
  lcd.print(F("TPS:"));
  lcd.setCursor(8, 0);
  lcd.print(F(" %"));
  // Display Map
  lcd.setCursor(0, 1);
  lcd.print(F("MAP:"));
  lcd.setCursor(8, 1);
  lcd.print(F(" KPA"));
  lcd.print(F(" ("));
  lcd.setCursor(17, 1);
  lcd.print(F(")"));
  // Display EGT
  lcd.setCursor(0, 2);
  lcd.print(F("EGT:"));
  lcd.setCursor(8, 2);
  lcd.print(F(" C"));
  // Display Cutoff
  lcd.setCursor(0, 3);
  lcd.print(F("CUT:"));

  while (true)
  {
    // Loop in menu
    keypad.loop(ButtonHandler); // Check keypad press
    if (button == keypad_select)
    {
      opState = OFF;
      return;
    }
    // Run master
    DoControl();
    // Update LCD
    if ((millis() - Last_LCD_Time) > LCD_Interval) {
      Last_LCD_Time = millis();
      
      // Display TPS
      lcd.setCursor(5, 0);
      padPrintLCD(tps, 3);
      // Display MAP
      lcd.setCursor(5, 1);
      padPrintLCD(mapsensor, 3);
      // boost_wanted
      lcd.setCursor(14, 1);
      padPrintLCD(boost_wanted, 3);
      // Display EGT
      lcd.setCursor(5, 2);
      padPrintLCD(egtsensor, 3);
      // Display Cutoff
      lcd.setCursor(4, 3);
      if (cutoff_act == 0)
      {
        lcd.print(F("  NO"));
      }
      if (cutoff_act == 1)
      {
        lcd.print(F(" YES"));
      }

    }
  }
}


// ************************************************
// State RAW(default)
// ************************************************
void Raw()
{

}


// ************************************************
// Execute the control loop
// ************************************************

void DoControl()
{
  // Canbuss
  canbuss_read();  // Check if data on canbuss, reply if instructed or set variables
}

// ************************************************
// Canbuss Read
//
// ************************************************

void canbuss_read() {

  int vala;
  int valb;

  // *********************
  // Read messages
  // *********************

  if (!digitalRead(CAN0_INT)) {              // If CAN0_INT pin is low, read receive buffer

    CAN0.readMsgBuf(&canbuss_rxId, &canbuss_len, canbuss_in);      // Read data: canbuss_len = data length, buf = data byte(s)

    // *******************
    // Listen to pushdata
    // *******************
    if (canbuss_rxId == 0x1F) //Check if remote id is HEX 1F (Push data)

    {

      //**************
      //Mapsensor
      //**************
      if (canbuss_in[0] == 1) {

        if (canbuss_in[1] == 1) mapsensor = canbuss_in[2];
        if (canbuss_in[1] == 10) mapsensor_raw = canbuss_in[2];

      }

      //**************
      //Egt
      //**************
      if (canbuss_in[0] == 2) {

        if (canbuss_in[1] == 1) egtsensor = (canbuss_in[2] * 8) + canbuss_in[3] ;
      }


      //**************
      //TPS
      //**************
      if (canbuss_in[0] == 4) {

        if (canbuss_in[1] == 1) tps = canbuss_in[2];
        if (canbuss_in[1] == 10) tps_raw = canbuss_in[2];

      }

      //**************
      //boost_control
      //**************
      if (canbuss_in[0] == 10) {

        if (canbuss_in[1] == 1) boost_wanted = canbuss_in[2];
      }

      //**************
      //cutoff
      //**************
      if (canbuss_in[0] == 100) {

        if (canbuss_in[1] == 1) cutoff_act = canbuss_in[2];
      }

    }

  }
}

// ************************************************
// Canbuss Send Message
//
// ************************************************
void canbuss_send(int canbuss_send_id, byte data0, byte data1, byte pid, byte data3, byte data4, byte data5, byte data6, byte data7) {

  // *********************
  // Send messages
  // *********************

  canbuss_out[0] = data0;
  canbuss_out[1] = data1;
  canbuss_out[2] = pid;
  canbuss_out[3] = data3;
  canbuss_out[4] = data4;
  canbuss_out[5] = data5;
  canbuss_out[6] = data6;
  canbuss_out[7] = data7;

  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  //  byte sndStat = CAN0.sendMsgBuf(canbuss_send_id, 0, 8, canbuss_send_data);
  byte sndStat = CAN0.sendMsgBuf(canbuss_send_id, 0, 8, canbuss_out);

  if (sndStat == CAN_OK) {
    Serial.println(F(" < Canbuss Reply Sent Successfully "));

  }
  else {
    Serial.println(F(" *** Error Sending Canbuss Message *** "));

  }
}

// ************************************************
// ButtonHandler
// Read keypress
// ************************************************
void ButtonHandler(const ButtonParam& param)
{

  // Klick left
  if ((param.button == 1) && param.state == 1)
  {
    button = 0x001;
  }

  if ((param.button == 2) && param.state == 1)
  {
    button = 0x002;
  }

  if ((param.button == 3) && param.state == 1)
  {
    button = 0x003;
  }

  if ((param.button == 4) && param.state == 1)
  {
    button = 0x004;
  }

  if ((param.button == 5) && param.state == 1)
  {
    button = 0x005;
  }

}

// ************************************************
// padPrintLCD (leading 0 or spaces)
// ************************************************
void padPrintLCD(int value, int width)
{
  // pads values with leading zeros to make the given width
  char valueStr[6]; // large enough to hold an int
  itoa (value, valueStr, 10);
  int len = strlen(valueStr);
  if (len < width) {
    len = width - len;
    while (len--)
      lcd.print(' ');
  }
  lcd.print(valueStr);
}
