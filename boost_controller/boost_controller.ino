

//------------------------------------------------------------------------------------------------
// Magnus Karlberg
//
//
//
//v 4.0
//------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------
// FIX
//
// Checksum canbuss set
//
//
//------------------------------------------------------------------------------------------------


//************************************************************************************************
// START
//************************************************************************************************

// ************************************************
// Include
// ************************************************

//EEPROM
#include <EEPROM.h>

// CAN MCP2515
#include <mcp_can.h>
#include <SPI.h>

// EGT Probe
#include "max6675.h"

// PID
#include <PID_v1.h>

// Servo
// #include <Servo.h>

// ************************************************
// Pin definitions
// ************************************************

// PWM
#define PWM_OUT 5           // PWM Output pin 5

// Read PWM (HW debug)
#define PWM_READ 13         // PWM Read pin 13

// EGT
// const byte ktcSO = 7;    // SO Digital PIN
// const byte ktcCS = 8;    // CS Digital PIN
// const byte ktcCLK = 9;   // CLK Digital PIN

// Canbuss
#define CAN0_INT 2          // Set INT to pin 2 (UNO)
MCP_CAN CAN0(10);           // Set CS to pin 10 (UNO)
// si   orange	ardunio icmpport	(pin 11 UNO)
// so		brun	  ardunio icmpport	(pin 12 UNO)
// sck	gul		  ardunio icmpport	(pin 13 UNO)

// MPX4250DP Preasure sensor
#define mpx4250dp A0        // EGT Sensor Analog PIN 0

// TPS
#define TPS_READ A1         // TPS Analog PIN 1


// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double PID_Setpoint;
double PID_Input;
double PID_Output;

// pid tuning parameters
double PID_Kp;
double PID_Ki;
double PID_Kd;

// Param to convert mapscale to 0-1023 analog scale
// double pid_conv_map_maxkpa;

//Specify the links and initial tuning parameters
PID myPID(&PID_Input, &PID_Output, &PID_Setpoint, PID_Kp, PID_Ki, PID_Kd, DIRECT);


// ************************************************
// Constants / Global variables
// ************************************************


// EEPROM addresses for persisted data

const byte PID_Kp_Address = 0;
const byte PID_Ki_Address = 8;
const byte PID_Kd_Address = 16;
const byte mapcal_min_Address = 24;
const byte mapcal_max_Address = 32;
const byte map_minkpa_Address = 40;
const byte map_maxkpa_Address = 48;
const byte ctrl_cutoff_Address = 56;
const byte cutoff_boost_Address = 64;
const byte cutoff_egt_Address = 72;
const byte use_static_boost_Address = 80;
const byte boost_static_Address = 88;
const byte boost_wanted_min_Address = 96;
const byte boost_wanted_max_Address = 104;
const byte tps_maxboost_Address = 112;
const byte tpscal_min_Address = 120;
const byte tpscal_max_Address = 128;
// const byte _Address = 136;

// TEST!!!!
int test = 0;

// Read PWM (debug)
static double duty;
static double freq;
static long highTime = 0;
static long lowTime = 0;
static long tempPulse;

// SerialCMD
String S_inData;
bool S_echo = 0;
bool S_debug_can = 0;
bool S_debug_sensor = 0;
bool S_debug_pwm = 0;

// Timers

const int Boostcontrol_Interval = 10;
unsigned long Last_Boostcontrol_Time = 0;

const int Canbuss_Push_Interval = 100;
unsigned long Last_Canbuss_Push_Time = 0;

const int Serial_Log_Interval = 250;
unsigned long Serial_Last_Log_Time = 0;


// Mapsensor
int mapsensor = 0;                    // Sensor reading
int mapcal_min;                       // MIN Raw calibration
int mapcal_max;                       // MAX Raw calibration
int map_minkpa;                       // MIN Sensor reading
int map_maxkpa;                       // Max Sensor reading

//EGT
int egtsensor = 0;                    // EGT Sensor reading

// Canbus
long unsigned int canbuss_rxId;
unsigned char canbuss_len = 0;
unsigned char canbuss_in[8];
unsigned char canbuss_out[8];
// char msgString[128];               // Array to store serial string

// Cutoff
bool ctrl_cutoff;               // Use Cutoff
int cutoff_boost;               // Cutoff at boost
int cutoff_egt;                 // Cutoff at egt
bool cutoff_act = 0;            // Cutoff activated

// TPS
int tps = 0;                    // Sensor reading (0-100%)
int tpscal_min;                 // MIN Raw calibration
int tpscal_max;                 // MAX Raw calibration

// Boostcontrol
bool use_static_boost;          // Use static boostvalue Yes/No
int boost_static;               // Static boost value (kpa)
int boost_wanted = 0;           // Wanted boost (kpa) (calculated)
int boost_wanted_min;           // Wanted minimum boost (kpa)
int boost_wanted_max;           // Wanted maximum boost (kpa)
int tps_maxboost;               // tps % when we want maximum boost


// ************************************************
// Init
// ************************************************

// EGT
MAX6675 ktc(9, 8, 7); // PIN def

// Servo
// Servo myservo;  // create servo object to control a servo

// ************************************************************************************************
// Setup
// ************************************************************************************************
void setup() {

  // ************************************************
  // Pinmodes
  // ************************************************

  // Canbuss
  pinMode(CAN0_INT, INPUT);

  // MAP
  pinMode(mpx4250dp, INPUT);

  // PWM
  pinMode(PWM_OUT, OUTPUT);
  pinMode(PWM_READ, INPUT);

  // TPS
  pinMode(TPS_READ, INPUT);

  // ************************************************
  // PWM Freq
  // ************************************************

  //Leonardo pin5 30hz PWM (16bit port)
  TCCR3B = 0B00000101;
  TCCR3A = 0B00000001;

  // ************************************************
  // Setup cont.
  // ************************************************

  // Start serial
  Serial.begin(115200);
  Serial.setTimeout(10000); // 10sek timeout
  // while (!Serial) {   // Wait for serial port to connect. Needed for native USB (Leonardo etc)
  //  ;
  // }
  Serial.println(F("Serial connection started."));

  // Canbuss
  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  Serial.println(F("Initializing MCP2515..."));
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println(F("MCP2515 Initialized Successfully!"));
  else Serial.println(F("Error Initializing MCP2515..."));

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted.
  // Set operation mode to normal so the MCP2515 sends acks to received data.

  // Load parameters
  LoadParameters();

  // Initialize the PID and related variables
  PID_Input = mapsensor;
  PID_Setpoint = boost_wanted;
  myPID.SetTunings(PID_Kp, PID_Ki, PID_Kd);
  myPID.SetMode(AUTOMATIC);

  //Servo
  //myservo.attach(5);  // attaches the servo on pin 5 to the servo object

  // Delay before loop
  delay(500);

  // Print help message
  Serial.println(F(""));
  Serial.println(F("Press h for help."));

}
// ************************************************************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************************************************************

void loop() {

  // *************************************
  // Canbuss Read
  // *************************************
  canbuss_read(); // Check if data on canbuss, reply if instructed or set variables

  // *************************************
  // Boostcontrol
  // *************************************
  if ((millis() - Last_Boostcontrol_Time) > Boostcontrol_Interval) {
    Last_Boostcontrol_Time = millis();

    // Get sensordata
    Get_EGT();                  // Get egt reading
    Get_Boost();                // Get map reading
    Get_TPS();                  // Get TPS reading
    Check_Cutoff();             // Check cutoff

    // PWM boostcontrol
    Boostcontrol();
  }

  // *************************************
  // Push Canbussdata
  // *************************************
  if ((millis() - Last_Canbuss_Push_Time) > Canbuss_Push_Interval) {
    Last_Canbuss_Push_Time = millis();

    // Send sensordata every Canbuss_Push_Interval
    canbuss_push_Sensordata();
  }

  // *************************************
  // Serial
  // *************************************
  if ((millis() - Serial_Last_Log_Time) > Serial_Log_Interval) {
    Serial_Last_Log_Time = millis();

    // Menu
    serialCMD();

    // Logging
    S_Log();
  }

}

// ************************************************************************************************
// Functions
//
//
// ************************************************************************************************


// ************************************************
// Canbuss Read
//
// ************************************************

void canbuss_read() {

  int vala;
  int valb;


  // *************************************
  // Read messages
  // *************************************

  if (!digitalRead(CAN0_INT)) {              // If CAN0_INT pin is low, read receive buffer

    CAN0.readMsgBuf(&canbuss_rxId, &canbuss_len, canbuss_in);      // Read data: canbuss_len = data length, canbuss_in = data byte(s)

    // *******************
    // Get / Reply sensordata
    // *******************
    if (canbuss_rxId == 0x1B) {

      //**************
      //Mapsensor
      //**************
      if (canbuss_in[0] == 1) {

        //**************
        //mapcal_min(raw)
        //**************
        if (canbuss_in[1] == 2) canbuss_send(0x1C, 1, 2, mapcal_min, 0x00, 0x00, 0x00, 0x00, 0x00);
        //**************
        //mapcal_max(raw)
        //**************
        if (canbuss_in[1] == 3)
        {
          if (mapcal_max < 8) vala = 0;
          else vala = (mapcal_max * 0.125);
          valb = (mapcal_max - (vala * 8));
          canbuss_send(0x1C, 1, 3, vala, valb, 0x00, 0x00, 0x00, 0x00);
        }
        //**************
        //map_minkpa
        //**************
        if (canbuss_in[1] == 4) canbuss_send(0x1C, 1, 4, map_minkpa, 0x00, 0x00, 0x00, 0x00, 0x00);
        //**************
        //map_maxkpa
        //**************
        if (canbuss_in[1] == 5) canbuss_send(0x1C, 1, 5, map_maxkpa, 0x00, 0x00, 0x00, 0x00, 0x00);
      }

      //**************
      //TPS
      //**************
      if (canbuss_in[0] == 4) {

        //**************
        //tpscal_min(raw)
        //**************
        if (canbuss_in[1] == 2) canbuss_send(0x1C, 4, 2, tpscal_min, 0x00, 0x00, 0x00, 0x00, 0x00);
        //**************
        //tpscal_max(raw)
        //**************
        if (canbuss_in[1] == 3)
        {
          if (tpscal_max < 8) vala = 0;
          else vala = (tpscal_max * 0.125);
          valb = (tpscal_max - (vala * 8));
          canbuss_send(0x1C, 4, 3, vala, valb, 0x00, 0x00, 0x00, 0x00);
        }
      }

      //**************
      //Boostcontrol
      //**************
      if (canbuss_in[0] == 10) {

        //**************
        //use_static_boost(ON/OFF)
        //**************
        if (canbuss_in[1] == 2) canbuss_send(0x1C, 10, 2, use_static_boost, 0x00, 0x00, 0x00, 0x00, 0x00);
        //**************
        //boost_static(kpa)
        //**************
        if (canbuss_in[1] == 3) canbuss_send(0x1C, 10, 3, boost_static, 0x00, 0x00, 0x00, 0x00, 0x00);
        //**************
        //boost_wanted_min(kpa)
        //**************
        if (canbuss_in[1] == 4) canbuss_send(0x1C, 10, 4, boost_wanted_min, 0x00, 0x00, 0x00, 0x00, 0x00);
        //**************
        //boost_wanted_max(kpa)
        //**************
        if (canbuss_in[1] == 5) canbuss_send(0x1C, 10, 5, boost_wanted_max, 0x00, 0x00, 0x00, 0x00, 0x00);
        //**************
        //tps_maxboost(%)
        //**************
        if (canbuss_in[1] == 6) canbuss_send(0x1C, 10, 6, tps_maxboost, 0x00, 0x00, 0x00, 0x00, 0x00);
      }

      //**************
      //cutoff
      //**************
      if (canbuss_in[0] == 100) {

        //**************
        //ctrl_cutoff(ON/OFF)
        //**************
        if (canbuss_in[1] == 2) canbuss_send(0x1C, 100, 2, ctrl_cutoff, 0x00, 0x00, 0x00, 0x00, 0x00);
        //**************
        //cutoff_boost
        //**************
        if (canbuss_in[1] == 3) canbuss_send(0x1C, 100, 3, cutoff_boost, 0x00, 0x00, 0x00, 0x00, 0x00);
        //**************
        //cutoff_egt
        //**************
        if (canbuss_in[1] == 4)
        {
          if (cutoff_egt < 8) vala = 0;
          else vala = (cutoff_egt * 0.125);
          valb = (cutoff_egt - (vala * 8));
          canbuss_send(0x1C, 100, 4, vala, valb, 0x00, 0x00, 0x00, 0x00);
        }
      }

    }

    // *******************
    // SET sensordata
    // *******************
    if (canbuss_rxId == 0x1A) {

      //**************
      //Mapsensor
      //**************
      if (canbuss_in[0] == 1) {

        //**************
        //mapcal_min(raw)
        //**************
        if (canbuss_in[1] == 2) mapcal_min = canbuss_in[2];
        //**************
        //mapcal_max(raw)
        //**************
        if (canbuss_in[1] == 3) mapcal_max = (canbuss_in[2] * 8) + canbuss_in[3] ;
        //**************
        //map_minkpa
        //**************
        if (canbuss_in[1] == 4) map_minkpa = canbuss_in[2];
        //**************
        //map_maxkpa
        //**************
        if (canbuss_in[1] == 5) map_maxkpa = canbuss_in[2];
      }

      //**************
      //TPS
      //**************
      if (canbuss_in[0] == 4) {

        //**************
        //tpscal_min(raw)
        //**************
        if (canbuss_in[1] == 2) tpscal_min = canbuss_in[2];
        //**************
        //tpscal_max(raw)
        //**************
        if (canbuss_in[1] == 3) tpscal_max = (canbuss_in[2] * 8) + canbuss_in[3] ;
      }

      //**************
      //Boostcontrol
      //**************
      if (canbuss_in[0] == 10) {

        //**************
        //use_static_boost(ON/OFF)
        //**************
        if ((canbuss_in[1] == 2) && ((canbuss_in[2] == 0) || (canbuss_in[2] == 1))) use_static_boost = canbuss_in[2];
        //**************
        //boost_static(kpa)
        //**************
        if (canbuss_in[1] == 3) boost_static = canbuss_in[2];
        //**************
        //boost_wanted_min(kpa)
        //**************
        if (canbuss_in[1] == 4) boost_wanted_min = canbuss_in[2];
        //**************
        //boost_wanted_max(kpa)
        //**************
        if (canbuss_in[1] == 5) boost_wanted_max = canbuss_in[2];
        //**************
        //tps_maxboost(%)
        //**************
        if ((canbuss_in[1] == 6) && ((canbuss_in[2] >= 0) && (canbuss_in[2] <= 100))) tps_maxboost = canbuss_in[2];
      }

      //**************
      //cutoff
      //**************
      if (canbuss_in[0] == 100) {

        //**************
        //ctrl_cutoff(ON/OFF)
        //**************
        if ((canbuss_in[1] == 2) && ((canbuss_in[2] == 0) || (canbuss_in[2] == 1))) ctrl_cutoff = canbuss_in[2];
        //**************
        //cutoff_boost
        //**************
        if (canbuss_in[1] == 3) cutoff_boost = canbuss_in[2];
        //**************
        //cutoff_egt
        //**************
        if (canbuss_in[1] == 4) cutoff_egt = (canbuss_in[2] * 8) + canbuss_in[3] ;
      }

      //**************
      //system
      //**************
      if (canbuss_in[0] == 200) {

        //**************
        //SaveParameters
        //**************
        if ((canbuss_in[1] == 1) && (canbuss_in[2] == 1)) SaveParameters();
        //**************
        //PID SetTunings
        //**************
        if ((canbuss_in[1] == 2) && (canbuss_in[2] == 1)) myPID.SetTunings(PID_Kp, PID_Ki, PID_Kd);
      }

    }


    // *************************************
    // Debug
    // *************************************

    // Print canbuss_in to Serial
    if (S_debug_can == 1) {
      Serial.println(F(""));
      Serial.print(F("ID: "));
      Serial.println(canbuss_rxId, HEX);
      for (byte i = 0; i < canbuss_len; i++) {
        Serial.print(F(" canbuss_in["));
        Serial.print(i);
        Serial.print(F("]:"));
        Serial.print(F(" (DEC) "));
        Serial.print(canbuss_in[i]);
        Serial.print(F(" (HEX) "));
        Serial.println(canbuss_in[i], HEX);
      }
      Serial.println(F(""));

      if (canbuss_rxId == 26) {    //Check if remote id is HEX 0x1A
        Serial.println(F(" > Canbuss Recevied Set "));
      }

      if (canbuss_rxId == 27) {    //Check if remote id is HEX 0x1B
        Serial.println(F(" > Canbuss Recevied Get "));
      }

      if (canbuss_rxId == 28) {    //Check if remote id is HEX 0x1C
        Serial.println(F(" > Canbuss Recevied Reply "));
      }
    }

  }
}


// ************************************************
// Canbuss Push Sensordata
//
// ************************************************

void canbuss_push_Sensordata() {

  int vala;
  int valb;
  int val0;

  // ********************
  // mapsensor
  // ********************

  //mapsensor(kpa)      A (0-255)       (sensor)
  canbuss_send(0x1F, 1, 1, mapsensor, 0x00, 0x00, 0x00, 0x00, 0x00);

  //mapsensor_raw      ((A*8)+B) (0-1023)  (sensor)
  val0 = analogRead(mpx4250dp);
  if (val0 < 8) vala = 0;
  else vala = (val0 * 0.125);
  valb = (val0 - (vala * 8));
  canbuss_send(0x1F, 1, 10, vala, valb, 0x00, 0x00, 0x00, 0x00);
  
  // ********************
  // egtsensor
  // ********************

  //egtsensor(celcius)    ((A*8)+B) (0-1023)  (sensor)
  if (egtsensor < 8) vala = 0;
  else vala = (egtsensor * 0.125);
  valb = (egtsensor - (vala * 8));
  canbuss_send(0x1F, 2, 1, vala, valb, 0x00, 0x00, 0x00, 0x00);

  // ********************
  // TPS
  // ********************

  //tps(%)          A (0-100)       (sensor)
  canbuss_send(0x1F, 4, 1, tps, 0x00, 0x00, 0x00, 0x00, 0x00);
  
  //tps_raw        ((A*8)+B) (0-1023)  (sensor)
  val0 = analogRead(TPS_READ);
  if (val0 < 8) vala = 0;
  else vala = (val0 * 0.125);
  valb = (val0 - (vala * 8));
  canbuss_send(0x1F, 4, 10, vala, valb, 0x00, 0x00, 0x00, 0x00);
  
  // ********************
  // Boostcontrol
  // ********************

  //boost_wanted(kpa)      A (0-255)       (sensor)
  canbuss_send(0x1F, 10, 1, boost_wanted, 0x00, 0x00, 0x00, 0x00, 0x00);

  // ********************
  // cutoff
  // ********************

  //cutoff_act(ON/OFF)    A (0-1)       (sensor)
  canbuss_send(0x1F, 100, 1, cutoff_act, 0x00, 0x00, 0x00, 0x00, 0x00);

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

  if (S_debug_can == 1) { // Serial debug
    if (sndStat == CAN_OK) {
      Serial.println(F(" Canbuss Message Sent Successfully "));

    }
    else {
      Serial.println(F(" *** Error Sending Canbuss Message *** "));

    }

  }

}

// ************************************************
// EGT
//
// ************************************************

void Get_EGT() {
  egtsensor = ktc.readCelsius();
}

// ************************************************
// Get Boost
//
// ************************************************
void Get_Boost() {

  mapsensor = analogRead(mpx4250dp);
  //mapsensor = ReadSens_and_Condition(mpx4250dp, 30);                          // Read avg / 30 times
  mapsensor = map(mapsensor, mapcal_min, mapcal_max, map_minkpa, map_maxkpa );  // Map input 0-250 kpa mpx4250dp
  mapsensor = constrain(mapsensor, map_minkpa, map_maxkpa);                     // Constrain for security

}

// ************************************************
// Get TPS
//
// ************************************************
void Get_TPS() {

  tps = analogRead(TPS_READ);
  //tps = ReadSens_and_Condition(TPS_READ, 30);         // Read avg / 30 times
  tps = map(tps, tpscal_min, tpscal_max, 0, 100 );      // Map input 0-100%
  tps = constrain(tps, 0, 100);                         // Constrain for security

}

// ************************************************
// Boostcontrol
// ************************************************
void Boostcontrol() {

  // Check Cutoff
  if (cutoff_act == 1) {

    boost_wanted = 0; // set output to 0 when cutoff is active

  }

  else {

    // Check use_static_boost
    if (use_static_boost == 1) {
      boost_wanted = boost_static;  // set wanted boost to static if use_static_boost is active
    }

    else {  // Else calculate from tps

      // Invert and set maxboost after tps setting ( max boost at 70% tps etc)
      boost_wanted = map(tps, 0, tps_maxboost, 0, boost_wanted_max );

      // Constrain in and set minimum boost
      boost_wanted = constrain( boost_wanted, boost_wanted_min, boost_wanted_max);
    }

  }
  // Boostcontrol
  PID_Input = mapsensor;
  PID_Setpoint = boost_wanted;
  myPID.Compute();

  // int Servo_Output = map(PID_Output, 0, 255, 1, 179);     // scale it to use it with the servo (value between 0 and 180)
  // myservo.write(Servo_Output);                            // sets the servo position according to the scaled value

  // PID_Output=255-PID_Output;
  analogWrite(PWM_OUT, map(PID_Output, 0, 255, 25, 210));     // Write PWM, remove the extremes (10%)

}

// ************************************************
// Check Cutoff
// ************************************************
void Check_Cutoff() {

  if ((ctrl_cutoff == 1) && ((mapsensor >= cutoff_boost) || (egtsensor >= cutoff_egt))) {
    cutoff_act = 1;
  }
  else {
    cutoff_act = 0;
  }
}

// ************************************************
// Smooth sensor
// ************************************************
int ReadSens_and_Condition(int val2, byte avg2) {

  byte i = 0;
  int sval = 0;

  for (i = 0; i < avg2; i++) {
    sval = sval + analogRead(val2);  // sensorport
  }

  sval = sval / avg2;    // average
  return sval;
}

// ************************************************
// Serial logging
//
// ************************************************

void S_Log() {

  if ( S_debug_sensor == 1) {
    Serial.print(F("MAPRAW = "));
    Serial.print(analogRead(mpx4250dp));
    Serial.print(F(" ,"));
    Serial.print(F("MAP = "));
    Serial.print(mapsensor);
    Serial.print(F(" ,"));
    Serial.print(F("TPSRAW = "));
    Serial.print(analogRead(TPS_READ));
    Serial.print(F(" ,"));
    Serial.print(F("TPS = "));
    Serial.print(tps);
    Serial.print(F(" ,"));
    Serial.print(F(" EGT = "));
    Serial.print(egtsensor);
    Serial.print(F(" ,"));
    Serial.print(F("cutoff_act = "));
    Serial.print(cutoff_act);
    Serial.print(F(" ,"));
    Serial.print(F("boost_wanted = "));
    Serial.print(boost_wanted);
    Serial.print(F(" ,"));
    Serial.print(F("PID_Input = "));
    Serial.print(PID_Input);
    Serial.print(F(" ,"));
    Serial.print(F("PID_Setpoint = "));
    Serial.print(PID_Setpoint);
    Serial.print(F(" ,"));
    Serial.print(F("PID_Output = "));
    Serial.println(PID_Output);
  }

  if ( S_debug_pwm == 1) {
    readPWM(PWM_READ);
    Serial.print(F("Frequency = "));
    Serial.print(freq);
    Serial.print(F("Hz"));
    Serial.print(F(" ,"));
    Serial.print(F("Duty Cycle = "));
    Serial.print(duty);
    Serial.println(F("%"));
  }

}

// ************************************************
// Debug/Read pwm
// ************************************************


void readPWM(int readPin) {
  highTime = 0;
  lowTime = 0;

  tempPulse = pulseIn(readPin, HIGH);
  if (tempPulse > highTime) {
    highTime = tempPulse;
  }

  tempPulse = pulseIn(readPin, LOW);
  if (tempPulse > lowTime) {
    lowTime = tempPulse;
  }

  freq = ((double) 1000000) / (double (lowTime + highTime));
  duty = (100 * (highTime / (double (lowTime + highTime))));
}

// ************************************************
// SerialCMD (Newline activated)
// ************************************************
void serialCMD() {

  int S_int;

  //   Send data only when you receive data:
  if (Serial.available() > 0) {

    // Read the incoming byte:
    char recieved = Serial.read();
    S_inData += recieved;

    // Check message after enter:
    if (recieved == '\n') {

      // Display Echo
      if ( S_echo == 1) {
        Serial.print(F("Local Echo: "));
        Serial.println(S_inData);
      }
      // Help Menu
      if (S_inData == "h\n") {
        serialp(1); // Print line
        Serial.println(F("Global"));
        serialp(1); // Print line
        Serial.println(F("h    Help"));
        Serial.println(F("e    Echo ON/OFF"));
        Serial.println(F("dc   Debug CAN ON/OFF"));
        Serial.println(F("ds   Debug Sensor ON/OFF"));
        Serial.println(F("dp   Debug PWM ON/OFF"));
        serialp(1); // Print line
        Serial.println(F("Show settings"));
        serialp(1); // Print line
        Serial.println(F("smap  Map"));
        Serial.println(F("stps  TPS"));
        Serial.println(F("sco   Cutoff"));
        Serial.println(F("sbc   Boostcontrol"));
        Serial.println(F("spid  PID Settings"));
        serialp(1); // Print line
        Serial.println(F("Set settings"));
        serialp(1); // Print line
        Serial.println(F("Smap  Map"));
        Serial.println(F("Stps  TPS"));
        Serial.println(F("Sco   Cutoff"));
        Serial.println(F("Sbc   Boostcontrol"));
        Serial.println(F("Spid  PID Settings"));
        serialp(1); // Print line
      }
      // Set Echo
      else if (S_inData == "e\n") {
        if (S_echo == 0) {
          S_echo = 1;
          Serial.println(F("Echo on"));
        }
        else {
          S_echo = 0;
          Serial.println(F("Echo off"));
        }
      }
      // Set Debug CAN
      else if (S_inData == "dc\n") {
        if (S_debug_can == 0) {
          S_debug_can = 1;
          Serial.println(F("Debug CAN on"));
        }
        else {
          S_debug_can = 0;
          Serial.println(F("Debug CAN off"));
        }
      }
      // Set Debug Sensor
      else if (S_inData == "ds\n") {
        if (S_debug_sensor == 0) {
          S_debug_sensor = 1;
          Serial.println(F("Debug Sensor on"));
        }
        else {
          S_debug_sensor = 0;
          Serial.println(F("Debug Sensor off"));
        }
      }
      else if (S_inData == "dp\n") {
        if (S_debug_pwm == 0) {
          S_debug_pwm = 1;
          Serial.println(F("Debug PWM on"));
        }
        else {
          S_debug_pwm = 0;
          Serial.println(F("Debug PWM off"));
        }
      }

      // *****************
      // Map
      // *****************

      // Show Map Settings
      else if (S_inData == "smap\n") {
        serialp(1); // Print line
        Serial.print(F("mapcal_min: "));
        Serial.println(mapcal_min);
        Serial.print(F("mapcal_max: "));
        Serial.println(mapcal_max);
        Serial.print(F("map_minkpa: "));
        Serial.println(map_minkpa);
        Serial.print(F("map_maxkpa: "));
        Serial.println(map_maxkpa);
        serialp(1); // Print line
      }

      // Set Map Settings
      else if (S_inData == "Smap\n") {
        serialp(1); // Print line
        serialp(6); // Print "Enter new value or 9999 to skip "
        serialp(1); // Print line

        //mapcal_min
        Serial.print(F("mapcal_min (raw): "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int < 255) && (S_int > 0)) {
          mapcal_min = S_int;
          Serial.println(mapcal_min);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print("0");
          serialp(3); // Print " Max: "
          Serial.println("255");
        }

        //mapcal_max
        Serial.print(F("mapcal_max (raw): "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int < 1023) && (S_int > 500)) {
          mapcal_max = S_int;
          Serial.println(mapcal_max);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print("500");
          serialp(3); // Print " Max: "
          Serial.println("1023");
        }

        //map_minkpa
        Serial.print(F("map_minkpa (kpa): "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int < 100) && (S_int >= 0)) {
          map_minkpa = S_int;
          Serial.println(map_minkpa);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print("0");
          serialp(3); // Print " Max: "
          Serial.println("100");
        }

        //map_maxkpa
        Serial.print(F("map_maxkpa (kpa): "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int < 500) && (S_int > 50)) {
          map_maxkpa = S_int;
          Serial.println(map_maxkpa);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print("50");
          serialp(3); // Print " Max: "
          Serial.println("500");
        }

        // Save and exit
        SaveParameters();
        serialp(7); // Print "*** Saved ***"
      }

      // *****************
      // TPS
      // *****************

      // Show TPS Settings
      else if (S_inData == "stps\n") {
        serialp(1); // Print line
        Serial.print(F("tpscal_min: "));
        Serial.println(tpscal_min);
        Serial.print(F("tpscal_max: "));
        Serial.println(tpscal_max);
        serialp(1); // Print line
      }

      // Set TPS Settings
      else if (S_inData == "Stps\n") {
        serialp(1); // Print line
        serialp(6); // Print "Enter new value or 9999 to skip "
        serialp(1); // Print line

        //tpscal_min
        Serial.print(F("tpscal_min (raw): "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int < 255) && (S_int > 0)) {
          tpscal_min = S_int;
          Serial.println(tpscal_min);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print("0");
          serialp(3); // Print " Max: "
          Serial.println("255");
        }

        //tpscal_max
        Serial.print(F("tpscal_max (raw): "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int < 1023) && (S_int > 500)) {
          tpscal_max = S_int;
          Serial.println(tpscal_max);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print("500");
          serialp(3); // Print " Max: "
          Serial.println("1023");
        }

        // Save and exit
        SaveParameters();
        serialp(7); // Print "*** Saved ***"
      }


      // *****************
      // EGT
      // *****************

      // Show EGT Settings
      else if (S_inData == "segt\n") {
        serialp(1); // Print line
        Serial.print(F("egtsensor: "));
        Serial.println(egtsensor);
        serialp(1); // Print line
      }

      // *****************
      // Cutoff
      // *****************

      // Show Cutoff Settings
      else if (S_inData == "sco\n") {
        serialp(1); // Print line
        Serial.print(F("ctrl_cutoff: "));
        Serial.println(ctrl_cutoff);
        Serial.print(F("cutoff_boost: "));
        Serial.println(cutoff_boost);
        Serial.print(F("cutoff_egt: "));
        Serial.println(cutoff_egt);
        serialp(1); // Print line
      }

      // Set Cutoff menu
      else if (S_inData == "Sco\n") {
        serialp(1); // Print line
        serialp(6); // Print "Enter new value or 9999 to skip "
        serialp(1); // Print line

        //ctrl_cutoff
        Serial.print(F("ctrl_cutoff (1=ON/0=OFF): "));
        S_int = Serial.parseInt();          //transfer number to num
        if (S_int == 1) {
          ctrl_cutoff = 1;
          Serial.println("1");
        }
        else if (S_int == 0 ) {
          ctrl_cutoff = 0;
          Serial.println("0");
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
        }

        //cutoff_boost
        Serial.print(F("cutoff_boost (kpa): "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int < map_maxkpa - 1) && (S_int > map_minkpa + 1)) {
          cutoff_boost = S_int;
          Serial.println(cutoff_boost);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print(map_minkpa + 1);
          serialp(3); // Print " Max: "
          Serial.println(map_maxkpa - 1);
        }

        //cutoff_egt
        Serial.print(F("cutoff_egt (C): "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int < 1000) && (S_int > 500)) {
          cutoff_egt = S_int;
          Serial.println(cutoff_egt);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print("500");
          serialp(3); // Print " Max: "
          Serial.println("1000");
        }

        // Save and exit
        SaveParameters();
        serialp(7); // Print "*** Saved ***"
      }

      // *****************
      // Boostcontrol
      // *****************

      // Show Settings
      else if (S_inData == "sbc\n") {
        serialp(1); // Print line
        Serial.print(F("use_static_boost: "));
        Serial.println(use_static_boost);
        Serial.print(F("boost_static: "));
        Serial.println(boost_static);
        Serial.print(F("boost_wanted_min: "));
        Serial.println(boost_wanted_min);
        Serial.print(F("boost_wanted_max: "));
        Serial.println(boost_wanted_max);
        Serial.print(F("tps_maxboost: "));
        Serial.println(tps_maxboost);
        serialp(1); // Print line
      }

      // Set Boostcontrol menu
      else if (S_inData == "Sbc\n") {
        serialp(1); // Print line
        serialp(6); // Print "Enter new value or 9999 to skip "
        serialp(1); // Print line

        //use_static_boost
        Serial.print(F("use_static_boost (1=ON/0=OFF): "));
        S_int = Serial.parseInt();          //transfer number to num
        if (S_int == 1) {
          use_static_boost = 1;
          Serial.println("1");
        }
        else if (S_int == 0 ) {
          use_static_boost = 0;
          Serial.println("0");
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
        }

        //boost_static
        Serial.print(F("boost_static (kpa): "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int < map_maxkpa - 1) && (S_int > map_minkpa + 1)) {
          boost_static = S_int;
          Serial.println(boost_static);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print(map_minkpa + 1);
          serialp(3); // Print " Max: "
          Serial.println(map_maxkpa - 1);
        }

        //boost_wanted_min
        Serial.print(F("boost_wanted_min (kpa): "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int < map_maxkpa - 1) && (S_int > map_minkpa + 1)) {
          boost_wanted_min = S_int;
          Serial.println(boost_wanted_min);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print(map_minkpa + 1);
          serialp(3); // Print " Max: "
          Serial.println(map_maxkpa - 1);
        }

        //boost_wanted_max
        Serial.print(F("boost_wanted_max (kpa): "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int < map_maxkpa - 1) && (S_int > map_minkpa + 1)) {
          boost_wanted_max = S_int;
          Serial.println(boost_wanted_max);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print(map_minkpa + 1);
          serialp(3); // Print " Max: "
          Serial.println(map_maxkpa - 1);
        }

        //tps_maxboost
        Serial.print(F("tps_maxboost (%): "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int <= 100) && (S_int >= 0)) {
          tps_maxboost = S_int;
          Serial.println(tps_maxboost);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print("0");
          serialp(3); // Print " Max: "
          Serial.println("100");
        }

        // Save and exit
        SaveParameters();
        serialp(7); // Print "*** Saved ***"
      }

      // *****************
      // PID
      // *****************

      // Show Settings
      else if (S_inData == "spid\n") {
        serialp(1); // Print line
        Serial.print(F("PID_Kp: "));
        Serial.println(PID_Kp);
        Serial.print(F("PID_Ki: "));
        Serial.println(PID_Ki);
        Serial.print(F("PID_Kd: "));
        Serial.println(PID_Kd);
        serialp(1); // Print line
      }

      // Set PID menu
      else if (S_inData == "Spid\n") {
        serialp(1); // Print line
        serialp(6); // Print "Enter new value or 9999 to skip "
        Serial.println(F("Input without dot, Example 100 = 1.00"));
        serialp(1); // Print line

        //PID_Kp
        Serial.print(F("PID_Kp : "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int <= 1000) && (S_int >= 0)) {
          PID_Kp = S_int * 0.01;
          Serial.println(PID_Kp);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print("0");
          serialp(3); // Print " Max: "
          Serial.println("1000");
        }

        //PID_Ki
        Serial.print(F("PID_Ki : "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int <= 1000) && (S_int >= 0)) {
          PID_Ki = S_int * 0.01;
          Serial.println(PID_Ki);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print("0");
          serialp(3); // Print " Max: "
          Serial.println("1000");
        }

        //PID_Kd
        Serial.print(F("PID_Kd : "));
        S_int = Serial.parseInt();          //transfer number to num
        if ((S_int <= 1000) && (S_int >= 0)) {
          PID_Kd = S_int * 0.01;
          Serial.println(PID_Kd);
        }
        else if (S_int == 9999 ) {
          serialp(4); // Print " Skipped"
        }
        else {
          Serial.print(S_int);
          serialp(5); // Print " Skipped *out of range*"
          serialp(2); // Print " Min: "
          Serial.print("0");
          serialp(3); // Print " Max: "
          Serial.println("1000");
        }

        // Save and exit
        myPID.SetTunings(PID_Kp, PID_Ki, PID_Kd);
        SaveParameters();
        serialp(7); // Print "*** Saved ***"
      }

      // *****************
      // Standard error / Clear
      // *****************

      // Standard error message
      //else {
      //  Serial.println("Command not found. Press h for help.");
      //}
      // Clear recieved buffer
      S_inData = "";
    }
  }
}

// ************************************************
// Serialprint (save memory/program storage space)
// ************************************************
int serialp(int val) {

  if ( val == 0 ) {
    Serial.print(F(","));
  }
  if ( val == 1 ) {
    Serial.println(F("--------------------------------"));
  }
  else if ( val == 2 ) {
    Serial.print(F(" Min: "));
  }
  else if ( val == 3 ) {
    Serial.print(F(" Max: "));
  }
  else if ( val == 4 ) {
    Serial.println(F("  Skipped"));
  }
  else if ( val == 5 ) {
    Serial.println(F(" Skipped *out of range*"));
  }
  else if ( val == 6 ) {
    Serial.println(F("Enter new value or 9999 to skip "));
  }
  else if ( val == 7 ) {
    Serial.println(F("*** Saved ***"));
  }
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{

  if (PID_Kp != EEPROM_readDouble(PID_Kp_Address))
  {
    EEPROM_writeDouble(PID_Kp_Address, PID_Kp);
  }
  if (PID_Ki != EEPROM_readDouble(PID_Ki_Address))
  {
    EEPROM_writeDouble(PID_Ki_Address, PID_Ki);
  }
  if (PID_Kd != EEPROM_readDouble(PID_Kd_Address))
  {
    EEPROM_writeDouble(PID_Kd_Address, PID_Kd);
  }
  if (mapcal_min != EEPROM_readDouble(mapcal_min_Address))
  {
    EEPROM_writeDouble(mapcal_min_Address, mapcal_min);
  }
  if (mapcal_max != EEPROM_readDouble(mapcal_max_Address))
  {
    EEPROM_writeDouble(mapcal_max_Address, mapcal_max);
  }
  if (map_minkpa != EEPROM_readDouble(map_minkpa_Address))
  {
    EEPROM_writeDouble(map_minkpa_Address, map_minkpa);
  }
  if (map_maxkpa != EEPROM_readDouble(map_maxkpa_Address))
  {
    EEPROM_writeDouble(map_maxkpa_Address, map_maxkpa);
  }
  if (ctrl_cutoff != EEPROM_readDouble(ctrl_cutoff_Address))
  {
    EEPROM_writeDouble(ctrl_cutoff_Address, ctrl_cutoff);
  }
  if (cutoff_boost != EEPROM_readDouble(cutoff_boost_Address))
  {
    EEPROM_writeDouble(cutoff_boost_Address, cutoff_boost);
  }
  if (cutoff_egt != EEPROM_readDouble(cutoff_egt_Address))
  {
    EEPROM_writeDouble(cutoff_egt_Address, cutoff_egt);
  }
  if (use_static_boost != EEPROM_readDouble(use_static_boost_Address))
  {
    EEPROM_writeDouble(use_static_boost_Address, use_static_boost);
  }
  if (boost_static != EEPROM_readDouble(boost_static_Address))
  {
    EEPROM_writeDouble(boost_static_Address, boost_static);
  }
  if (boost_wanted_min != EEPROM_readDouble(boost_wanted_min_Address))
  {
    EEPROM_writeDouble(boost_wanted_min_Address, boost_wanted_min);
  }
  if (boost_wanted_max != EEPROM_readDouble(boost_wanted_max_Address))
  {
    EEPROM_writeDouble(boost_wanted_max_Address, boost_wanted_max);
  }
  if (tps_maxboost != EEPROM_readDouble(tps_maxboost_Address))
  {
    EEPROM_writeDouble(tps_maxboost_Address, tps_maxboost);
  }
  if (tpscal_min != EEPROM_readDouble(tpscal_min_Address))
  {
    EEPROM_writeDouble(tpscal_min_Address, tpscal_min);
  }
  if (tpscal_max != EEPROM_readDouble(tpscal_max_Address))
  {
    EEPROM_writeDouble(tpscal_max_Address, tpscal_max);
  }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
  PID_Kp = EEPROM_readDouble(PID_Kp_Address);
  PID_Ki = EEPROM_readDouble(PID_Ki_Address);
  PID_Kd = EEPROM_readDouble(PID_Kd_Address);
  mapcal_min = EEPROM_readDouble(mapcal_min_Address);
  mapcal_max = EEPROM_readDouble(mapcal_max_Address);
  map_minkpa = EEPROM_readDouble(map_minkpa_Address);
  map_maxkpa = EEPROM_readDouble(map_maxkpa_Address);
  ctrl_cutoff = EEPROM_readDouble(ctrl_cutoff_Address);
  cutoff_boost = EEPROM_readDouble(cutoff_boost_Address);
  cutoff_egt = EEPROM_readDouble(cutoff_egt_Address);
  use_static_boost = EEPROM_readDouble(use_static_boost_Address);
  boost_static = EEPROM_readDouble(boost_static_Address);
  boost_wanted_min = EEPROM_readDouble(boost_wanted_min_Address);
  boost_wanted_max = EEPROM_readDouble(boost_wanted_max_Address);
  tps_maxboost = EEPROM_readDouble(tps_maxboost_Address);
  tpscal_min = EEPROM_readDouble(tpscal_min_Address);
  tpscal_max = EEPROM_readDouble(tpscal_max_Address);

  // Set default if corrupt eeprom

  if (isnan(PID_Kp))
  {
    PID_Kp = 1;
  }
  if (isnan(PID_Ki))
  {
    PID_Ki = 0;
  }
  if (isnan(PID_Kd))
  {
    PID_Kd = 0;
  }
  if (isnan(mapcal_min))
  {
    mapcal_min = 30;
  }
  if (isnan(mapcal_max))
  {
    mapcal_max = 940;
  }
  if (isnan(map_minkpa))
  {
    map_minkpa = 0;
  }
  if (isnan(map_maxkpa))
  {
    map_maxkpa = 250;
  }
  if (isnan(ctrl_cutoff))
  {
    ctrl_cutoff = 0;
  }
  if (isnan(cutoff_boost))
  {
    cutoff_boost = 150;
  }
  if (isnan(cutoff_egt))
  {
    cutoff_egt = 750;
  }
  if (isnan(use_static_boost))
  {
    use_static_boost = 1;
  }
  if (isnan(boost_static))
  {
    boost_static = 100;
  }
  if (isnan(boost_wanted_min))
  {
    boost_wanted_min = 0;
  }
  if (isnan(boost_wanted_max))
  {
    boost_wanted_max = 150;
  }
  if (isnan(tps_maxboost))
  {
    tps_maxboost = 70;
  }
  if (isnan(tpscal_min))
  {
    tpscal_min = 30;
  }
  if (isnan(tpscal_max))
  {
    tpscal_max = 940;
  }

}

// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    EEPROM.write(address++, *p++);
  }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
  double value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    *p++ = EEPROM.read(address++);
  }
  return value;
}
