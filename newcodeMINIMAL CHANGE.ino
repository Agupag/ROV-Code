
/****** Set Payload ID# *******/
const uint8_t payloadNumber = 2;        // integer from 1 through 12

/********** Tunables**************************/
#define SERIAL_BAUD 115200
const bool windowIsFront = true;        // set false for reversed payload mount
const long loggingInterval = 1000;      // milliseconds
const int displayFlip = 3;              // number of logs between line 2 flips
const String fname = "ROVDATA.CSV";     // SD card file name

/********** ConstantS*************************/

// ============================== ALTERED FROM ORIGINAL SHORECODE - BLOCK 2 START ===================== 
// Change these to m1, m2, m3, m4
#define T_VERT    m1
#define T_REAR_L  m2
#define T_REAR_R  m3
#define T_FRONT   m4

// --- Servo Limits ---
#define SERVO1_MIN_ANGLE 0    // Down direction limit (0-90)
#define SERVO1_MAX_ANGLE 180  // Up direction limit (90-180)
// ============================== ALTERED FROM ORIGINAL SHORECODE - BLOCK 2 END ===================== 

/**** Bools *******/
bool diagnostics = true;
bool usingServo1 = true;
bool usingServo2 = false; // Only using one servo //value altered from shorecode

/********** Libraries*************************/
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <printf.h>                 // rf24 print routine
#include <Wire.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>
#include <Servo.h> // SIMULATION ONLY - For visualizing the angle locally //changed from shorecode


/***********Pin Assignments********************/
#define CE_PIN 8                    // Nano nRF24 CE //value altered from shorecode
#define CSN_PIN 2                   // Nano nRF24 CSN //value altered from shorecode
#define LCD_ADDR 0x27               //value altered from shorecode

// ============================== ALTERED FROM ORIGINAL SHORECODE - BLOCK 3 START ===================== 
// Joystick pin layout for this repo's Nano wiring.
// No joystick buttons are used.
#define ANALOG_X_PIN A2             // joystick 1 horizontal
#define ANALOG_Y_PIN A3             // joystick 1 vertical
#define ANALOG_X2_PIN A6            // joystick 2 horizontal
#define ANALOG_Y2_PIN A7            // joystick 2 vertical

#define ANALOG_X_CORRECTION 128
#define ANALOG_Y_CORRECTION 128
#define ANALOG_X2_CORRECTION 128
#define ANALOG_Y2_CORRECTION 128

#define BUTTON_UP_PIN 3             // D3 for UP fast button
#define BUTTON_DOWN_PIN 4           // D4 for DOWN fast button

const int chipSelect = 10;          // safe SD CS pin for Nano wiring

// Timeout for radio.available() poll (milliseconds).
// Prevents hanging forever if the payload side is not responding.
#define RADIO_TIMEOUT_MS 5000
// ============================== ALTERED FROM ORIGINAL SHORECODE - BLOCK 3 END ===================== 


/************Data Structures******************/
struct payload_data           //upbound data struct - from payload to surface
{
  const uint16_t startmark = 0xFFFF;
  uint16_t badPktCnt;
  uint16_t yaw;               // deg -90->90 - add 90 to use unsigned var
  uint16_t pitch;             // deg -180->180 - add 180 to use unsigned var
  uint16_t roll;              // deg 0-360
  uint16_t vlt;               // Voltage Vx100 (retain 2 sig figs)
  uint16_t amp;               // current Ax100 (retain 2 sig figs)
  uint16_t pres;              // Pressure - cm H20
  uint16_t temp;              // temp degCx100 as uint16_t
  uint16_t yr;
  uint16_t mon;
  uint16_t day;
  uint16_t hr;
  uint16_t min;
  uint16_t sec;
  uint16_t tds;               // water intrusion detection
} p_data = {};

struct control_data           // down bound data to payload from surface
{
  const uint16_t startmark = 0xFFFF;
  uint16_t crc;
  uint16_t s1angle = 90;      // 0-180 deg
  uint16_t s2angle = 90;
  uint16_t m1;                // 0=off, 1=forward, 2=reverse
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} c_data = {};

/*********Objects***************************/
RF24 radio( CE_PIN, CSN_PIN, 2000000 );               //SPI freqs must be 16mHz/2^n
LiquidCrystal_I2C lcd1( LCD_ADDR, 16, 2 );
File datafile;                                        // create file handle
Servo simServo;                                       // SIMULATION ONLY //changed from shorecode

/************Global Variables******************/
uint16_t * const pdwordptr = (uint16_t*)&p_data;
uint16_t * const cdwordptr = (uint16_t*)&c_data;
size_t sizeof_p_data;
size_t sizeof_c_data;
long lastPrintTime;
long curPrintTime;
bool SDPresent = false;                               // if no SD just carry-on
int printCounter = 1;                                 // print 2nd or 3rd line?
byte nodeAddress[5] = {};                             // pipe address:set from payload #
unsigned long loopCount = 0;                          // track loop iterations


/***CODE************************************************/
void setup()
{
// ============================== ALTERED FROM ORIGINAL SHORECODE - BLOCK 4 START ===================== 
  pinMode(ANALOG_X_PIN, INPUT);
  pinMode(ANALOG_Y_PIN, INPUT);
  pinMode(ANALOG_X2_PIN, INPUT);
  pinMode(ANALOG_Y2_PIN, INPUT);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);

  simServo.attach(5); // D5 - SIMULATION ONLY
// ============================== ALTERED FROM ORIGINAL SHORECODE - BLOCK 4 END ===================== 

  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CSN_PIN, HIGH);
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);

  lastPrintTime = millis();                       // start log display timer
  sizeof_p_data = sizeof( struct payload_data );
  sizeof_c_data = sizeof( struct control_data );
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  // ---- Unconditional milestone prints ----
  Serial.println(F("=== ROV Shore Controller Booting ==="));
  Serial.print(F("payload_data size: ")); Serial.println(sizeof_p_data);
  Serial.print(F("control_data size: ")); Serial.println(sizeof_c_data);
  Serial.print(F("Payload Number: ")); Serial.println(payloadNumber);

  Serial.println(F("[INIT] LCD..."));
  lcd1.init();
  lcd1.backlight();
  lcd1.clear();
  lcd1.print("SystemReady");
  Serial.println(F("[INIT] LCD OK"));

  delay(1000);

  Serial.println(F("[INIT] RF24..."));
  bool radioOk = initializeRF24();
  if( radioOk )
  {
    Serial.println(F("[INIT] RF24 OK"));
  }
  else
  {
    Serial.println(F("[INIT] *** RF24 FAILED *** - check wiring CE/CSN/SPI"));
  }
   
  delay(1000);

  Serial.println(F("[INIT] SD card..."));
  if(SD.begin(chipSelect))
  {
    SDPresent = true;
    Serial.println(F("[INIT] SD card found"));
    setupDataFile();
  }
  else
  {
    Serial.println(F("[INIT] No SD card - continuing without"));
    SDPresent = false;
  }

  // Always print RF24 details for diagnosis
  printf_begin();
  radio.printDetails();

  Serial.println(F("[INIT] Loading ACK payload..."));
  radio.writeAckPayload(1, &c_data, sizeof_c_data );  // preloadACK buffer
  Serial.println(F("[INIT] Starting listener..."));
  radio.startListening();                             // listen for payload!!
  Serial.println(F("[INIT] === SETUP COMPLETE - entering loop ==="));
}

/*********************************************************/
void loop()
/* The RF24 will send the return buffer automatically each time it
 * receives an input message. Very convenient.
 * - wait for data
 * - scan the control interface and generate the next control data set
 * - process the received data for display
 */
{
  loopCount++;
  radioCheckAndReply();
  refreshControlData();
  processPayloadData();
}

/*********************************************************
*    FUNCTIONS
*********************************************************/
bool initializeRF24()
{
  bool ok = radio.begin();
  radio.setPALevel(RF24_PA_LOW);           //power options LOW, HIGH, MAX
  radio.setDataRate(RF24_250KBPS);         //rate options to 2Mbps
  radio.enableAckPayload();                //enable slave reply
  uint8_t channel = ( payloadNumber -1 ) * 7 + 10;     //space channels between payloads
  radio.setChannel( channel );
  Serial.print(F("  RF24 channel: ")); Serial.println(channel);
  for ( int i = 0; i < 5; i++ ) nodeAddress[i] = channel;
  radio.openReadingPipe(1, nodeAddress);   //nodeAddress must match at each end
  Serial.print(F("  RF24 begin() returned: ")); Serial.println(ok ? "OK" : "FAIL");
  return ok;
}

/*************************************************************/
void radioCheckAndReply(void)
/* Wait for data from payload with timeout.
 * Prints status periodically so user can see what is happening
 * if the payload side is not responding.
 */
{
  unsigned long start = millis();
  unsigned long lastStatus = start;

  while(!radio.available())
  {
    unsigned long now = millis();

    // Print status every second while waiting
    if( now - lastStatus >= 1000 )
    {
      Serial.print(F("[RADIO] Waiting for payload... ("));
      Serial.print((now - start) / 1000);
      Serial.println(F("s)"));
      lastStatus = now;
    }

    // Timeout check
    if( now - start >= RADIO_TIMEOUT_MS )
    {
      Serial.println(F("[RADIO] *** TIMEOUT - no data from payload ***"));
      return;
    }
    delay(1);
  }

  radio.read( &p_data, sizeof_p_data );      // read payload data

  Serial.print(F("[RADIO] RX #"));
  Serial.print(loopCount);
  Serial.print(F(" | Yaw="));  Serial.print(p_data.yaw);
  Serial.print(F(" Pitch=")); Serial.print((int)p_data.pitch - 90);
  Serial.print(F(" Roll="));  Serial.print((int)p_data.roll - 180);
  Serial.print(F(" Pres="));  Serial.print(p_data.pres);
  Serial.print(F("cm V="));   Serial.print(p_data.vlt / 100.0, 1);
  Serial.print(F("V A="));    Serial.print(p_data.amp / 100.0, 1);
  Serial.print(F("A T="));    Serial.print(p_data.temp / 100);
  Serial.println(F("C"));

  if( diagnostics )
  {
    serialPrintBuf( (uint16_t *)&p_data, sizeof_p_data/2 );
  }
}

/*************************************************************/
void refreshControlData()    //generate new control data
{
  readControlSettings();
  radio.writeAckPayload(1, &c_data, sizeof_c_data );
}

// ============================== ALTERED FROM ORIGINAL SHORECODE - BLOCK 5 START ===================== 
/*************************************************************/
byte readAnalogAxisLevel(int pin)
{
  return map(analogRead(pin), 0, 1023, 0, 255);
}

/*************************************************************/
short readCenteredAxis(int pin, int correction)
{
  return readAnalogAxisLevel(pin) - correction;
}
// ============================== ALTERED FROM ORIGINAL SHORECODE - BLOCK 5 END ===================== 

// ============================== ALTERED FROM ORIGINAL SHORECODE - BLOCK 6 START ===================== 
/************************************************************/
void readControlSettings()
{
  short x1 = readCenteredAxis(ANALOG_X_PIN, ANALOG_X_CORRECTION);
  short y1 = readCenteredAxis(ANALOG_Y_PIN, ANALOG_Y_CORRECTION);
  short y2 = readCenteredAxis(ANALOG_Y2_PIN, ANALOG_Y2_CORRECTION);
  const char *movement = "neutral";

  // Reset all to off
  c_data.m1 = 0;
  c_data.m2 = 0;
  c_data.m3 = 0;
  c_data.m4 = 0;

  // --- Vertical Thruster Logic (D3 & D4 Buttons) ---
  bool btnUp = (digitalRead(BUTTON_UP_PIN) == LOW);
  bool btnDown = (digitalRead(BUTTON_DOWN_PIN) == LOW);

  if (btnUp && !btnDown) {
    c_data.T_VERT = 1; // forward/up
    movement = "up";
  } else if (btnDown && !btnUp) {
    c_data.T_VERT = 2; // reverse/down
    movement = "down";
  }

  // --- Horizontal Thruster Logic (Joystick 1) ---
  // Configuration: 2 rear at 45 degrees, 1 front straight
  if (y1 < -20) {
    if (x1 > 20) {
      c_data.T_REAR_L = 1; c_data.T_REAR_R = 0; c_data.T_FRONT = 1;
      if (c_data.T_VERT == 0) movement = "fwdRight";
    } else if (x1 < -20) {
      c_data.T_REAR_L = 0; c_data.T_REAR_R = 1; c_data.T_FRONT = 1;
      if (c_data.T_VERT == 0) movement = "fwdLeft";
    } else {
      c_data.T_REAR_L = 1; c_data.T_REAR_R = 1; c_data.T_FRONT = 1;
      if (c_data.T_VERT == 0) movement = "forward";
    }
  } else if (y1 > 20) {
    if (x1 > 20) {
      c_data.T_REAR_L = 2; c_data.T_REAR_R = 0; c_data.T_FRONT = 2;
      if (c_data.T_VERT == 0) movement = "revRight";
    } else if (x1 < -20) {
      c_data.T_REAR_L = 0; c_data.T_REAR_R = 2; c_data.T_FRONT = 2;
      if (c_data.T_VERT == 0) movement = "revLeft";
    } else {
      c_data.T_REAR_L = 2; c_data.T_REAR_R = 2; c_data.T_FRONT = 2;
      if (c_data.T_VERT == 0) movement = "reverse";
    }
  } else {
    if (x1 > 20) {
      c_data.T_REAR_L = 1; c_data.T_REAR_R = 2; c_data.T_FRONT = 0;
      if (c_data.T_VERT == 0) movement = "right";
    } else if (x1 < -20) {
      c_data.T_REAR_L = 2; c_data.T_REAR_R = 1; c_data.T_FRONT = 0;
      if (c_data.T_VERT == 0) movement = "left";
    }
  }

  // --- Servo Logic (Joystick 2 Y-axis) ---
  if( usingServo1 ) c_data.s1angle = constrain(map(y2, -128, 127, SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE), SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE);
  else c_data.s1angle = 90;

  if( usingServo2 ) c_data.s2angle = constrain(map(y2, -128, 127, 0, 180), 45, 120);
  else c_data.s2angle = 90;
  
  simServo.write(c_data.s1angle); // SIMULATION ONLY: Visually debug the angle locally

  // Always print control state for diagnosis
  Serial.print(F("[CTRL] "));
  Serial.print(movement);
  Serial.print(F(" | M1="));  Serial.print(c_data.m1);
  Serial.print(F(" M2="));    Serial.print(c_data.m2);
  Serial.print(F(" M3="));    Serial.print(c_data.m3);
  Serial.print(F(" M4="));    Serial.print(c_data.m4);
  Serial.print(F(" | S1="));  Serial.print(c_data.s1angle);
  Serial.print(F(" S2="));    Serial.println(c_data.s2angle);

  if( diagnostics )
  {
    Serial.print(F("  x1=")); Serial.print(x1);
    Serial.print(F(" y1=")); Serial.print(y1);
    Serial.print(F(" btnUp=")); Serial.print(btnUp);
    Serial.print(F(" btnDown=")); Serial.print(btnDown);
    Serial.print(F(" y2=")); Serial.println(y2);
  }
}
// ============================== ALTERED FROM ORIGINAL SHORECODE - BLOCK 6 END ===================== 

/*********************************************************/
void processPayloadData()
/* log data if SD present and Print to LCD on logging Interval
 */
{
  curPrintTime = millis();
  if( curPrintTime - lastPrintTime > loggingInterval )
  {
    printoLCD();
    if (SDPresent) writeLineToFile();
    lastPrintTime = curPrintTime;
  } 
}

/************************************************************/
void printoLCD()
/*
 * Print data selection from payload on LCD. This may require tweaking
 * based on what is going to fit.
 * Always print Yaw, Pitch, Roll, depth on line 1
 * 2nd line alternates for rest of data
 */
{
  lcd1.clear();
  lcd1.print( String( p_data.yaw));
  lcd1.print( " " );
  if ( windowIsFront )
  {
    lcd1.print( String( ((int)p_data.pitch) - 90 ) );
    lcd1.print( " " );
    lcd1.print( String( ((int)p_data.roll) - 180  ) );
  }
  else
  {
    lcd1.print( String( ( 90 - (int)p_data.pitch) ) );
    lcd1.print( " " );
    lcd1.print( String( ( 180 - (int)p_data.roll) ) );
  }
  lcd1.print( " " );
  lcd1.print( String( p_data.pres) );
  lcd1.print( "cm" );
  lcd1.setCursor(0,1);
  if( printCounter <= displayFlip )
  {
    lcd1.print( String( (float)(p_data.vlt/100.0),1 ) );
    lcd1.print( "v " );
    lcd1.print( String( (float)(p_data.amp/100.0),1 ) );
    lcd1.print( "a " );
    lcd1.print( String( p_data.temp/100) );
    lcd1.print( "C" );
    // cursor cosmetic //changed from shorecode
    printCounter++ ;
  }
  else
  {
    lcd1.print( String( p_data.hr ) );
    lcd1.print( ":" );
    lcd1.print( String( p_data.min ) );
    lcd1.print( ":" );
    lcd1.print( String( p_data.sec ) );
    lcd1.print( " " );
    lcd1.print( String( p_data.tds ) );
    lcd1.print( "mho" );
    // cursor cosmetic //changed from shorecode
    if ( ++printCounter > 2 * displayFlip ) printCounter = 1;
   }
}
/*********************************************************/
void serialPrintBuf( uint16_t * const bufptr, size_t bufsize )
/* Print either control or payload data set for diagnositic purposes
 */
{
  if( diagnostics )
  {
    Serial.println(F("Data in the buffer is: ")); //value altered from shorecode
    for( size_t i = 0; i < bufsize; i++ )
    {
     Serial.print( *(bufptr + i) );
     Serial.print(" / ");
    }
  Serial.println("");
  }
}

/*****************************************************/
void setupDataFile()
/* open the SD and write the header line
 */
{
  if( datafile = SD.open( fname, FILE_WRITE ) )
  {
    datafile.println("Yaw,Pitch,Roll,mV,mA,pres,degCx10,Yr,Mo,Day,Hr,Min,Sec");
    datafile.flush();
    Serial.println(F("[SD] Header written")); //value altered from shorecode
  }
  else
  {
    Serial.println(F("[SD] Failed to open file - disabling SD")); //value altered from shorecode
    SDPresent = false;
  }
}

/*********************************************************/
void writeLineToFile()
/* write data to SD, correct pitch and roll to 'true' values,
 * leave temp as 100x - don't want floats
 */
{
  String dataline = "";
  dataline += String(p_data.yaw);
  dataline += ",";
  if( windowIsFront )
  {
    dataline += String((int)p_data.pitch - 90);     // recast to signed
    dataline += ",";
    dataline += String((int)p_data.roll - 180);     // recast to signed
  }
  else
  {
    dataline += String( 90 - (int)p_data.pitch );   // recast to signed //value altered from shorecode
    dataline += ",";
    dataline += String( 180 - (int)p_data.roll );   // recast to signed //value altered from shorecode
  }
  dataline += ",";
  dataline += String(p_data.vlt * 10);          // x 10 gives mV, mA
  dataline += ",";
  dataline += String(p_data.amp * 10);
  dataline += ",";
  dataline += String(p_data.pres);
  dataline += ",";
  dataline += String(p_data.temp);
  dataline += ",";
  dataline += String(p_data.yr);
  dataline += ",";
  dataline += String(p_data.mon);
  dataline += ",";
  dataline += String(p_data.day);
  dataline += ",";
  dataline += String(p_data.hr);
  dataline += ",";
  dataline += String(p_data.min);
  dataline += ",";
  dataline += String(p_data.sec);
  datafile.println( dataline );
  datafile.flush();
  if( diagnostics ) Serial.println(F("[SD] Data written")); //value altered from shorecode
}

/*********************************************************/
