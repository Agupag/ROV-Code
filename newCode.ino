/* 
* Last Rev 1/19/2024:  Target: ArduinoNano/Every
* Shore Control Programming for ROV V4.
*
* IMPORTANT!! Set 'payloadNumber' at LINE 47 to the number of your payload,
* then compile and load this program to your shore controller or else your
* controller WILL NOT COMMUNICATE with your payload!!!
*
* CODE DESCRIPTION: The shore controller must send control data for 4 thrusters
* and optionally for 2 servos. The motor control values are '0' for off, '1' for
* forward, and '2' for reverse. Servo position is sent as an angle
* from 0 - 180 deg. the conventional scheme is to use 8 digital inputs on
* the arduino to read the positions of 8 SPST or 4 SPDT switches, but other
* schemes are possible as long as the correct resulting data is sent to the
* payload - i.e., a  "0","1" or "2" for each motor.
* SEE function "readControlSettings()"
*
* To insure the start word for the serial link cannot appear in the data set,
* all the data quantities (uint16_t) must be(will be) less than 14 bits.
* Thus we can bit shift them all one left. Every LSBbyte will be even and
* so will have a zero LS bit, every MSByte will still be less than 128 and
* will have zero MSbit. Thus we guarantee FF cannot appear in the data and
* so can be used as the buffer start marker. Reads are fixed lenghth - so
* don't need stop marker.
*
* Since conventional Euler pitch and roll values can be negative, we add
* 90 and 180 to them respectively in transport so all quants can be unsigned.
* Additionally, roll needs to be inverted as the BNO055 IMU is apparently
* designed to work upside down
*
* Yaw, pitch, roll, and timestamp (UTC) appear on an 16x2 LCD. Other data
* options include: battery voltage, battery current, conductivity
* cell (is vessel flodding?!), bad data count, system Temp. Display
* update rate is controlled by the 'loggingInterval' constant, but the rate of
* data arrival is controlled at the payload. Your shore control can log all
* the data to an SD card if present. A stopped clock
* readout means loss of the data link. The payload will shut down the thrusters
* within a few seconds if the link fails (defaut is 4 seconds).
*
* The 'diagnostics' flag controls all serial printing. It can be set to false
* once everything works and the control unit disconnected from any programming
* computer.
*
*/

/****** Set Payload ID# *******/
const uint8_t payloadNumber = 2;        // integer from 1 through 12

/********** Tunables**************************/
#define SERIAL_BAUD 115200
#define MPINCOUNT 8
const bool windowIsFront = true;        // set false for reversed payload mount
const long loggingInterval = 1000;      // milliseconds
const int displayFlip = 3;              // number of logs between line 2 flips
const String fname = "ROVDATA.CSV";     // SD card file name

/********** ConstantS*************************/

/**** Bools *******/
bool diagnostics = false;
bool usingServo1 = true;
bool usingServo2 = true;

/********** Libraries*************************/
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <printf.h>                 // rf24 print routine
#include <Wire.h>
#include <LCDIC2.h>
#include <SD.h>


/***********Pin Assignments********************/
#define CE_PIN 20                   // chip enable. D14 = A0
#define CSN_PIN 2                  // chip select for Radio SPI
#define LCD_ADDR 0x27
//#define AFOR 2                      // motor control pins
//#define AREV 3
/*#define BFOR 4
#define BREV 5
#define CFOR 6
#define CREV 7
#define DFOR 8
#define DREV 9*/

#define SV1READ A0
#define SV2READ A1
const int chipSelect = A7;                          // SD reader SPI CS pin

// Joystick input pins (from joystickCode.ino)
#define ANALOG_X_PIN A0
#define ANALOG_Y_PIN A1
//#define ANALOG_BUTTON_PIN 2
#define ANALOG_X2_PIN A2
#define ANALOG_Y2_PIN A3

// Default values when axis not actioned
#define ANALOG_X_CORRECTION 128
#define ANALOG_Y_CORRECTION 128
#define ANALOG_X2_CORRECTION 128
#define ANALOG_Y2_CORRECTION 128

#define BUTTON_UP_PIN 3
#define BUTTON_DOWN_PIN 4


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
LCDIC2 lcd1( LCD_ADDR, 16, 2 );
File datafile;                                        // create file handle

/************Global Variables******************/
//uint8_t mPins[8] = { AFOR, AREV, BFOR, BREV, CFOR, CREV, DFOR, DREV };
//bool mData[8] = {};
uint16_t * const pdwordptr = (uint16_t*)&p_data;   // byte pointer for struct
uint16_t * const cdwordptr = (uint16_t*)&c_data;
size_t sizeof_p_data;
size_t sizeof_c_data;
long lastPrintTime;
long curPrintTime;
bool SDPresent = false;                             // if no SD just carry-on
int printCounter = 1;                               // print 2nd or 3rd line?
byte nodeAddress[5] = {};                  // pipe address:set from payload #

struct button {
  byte pressed = 0;
};

struct analog {
  short x, y;
  button button;
};

// Virtual control states for AFOR/AREV/BFOR/... semantics.
// HIGH = released, LOW = pressed.
byte AFOR_state = HIGH;
byte AREV_state = HIGH;
byte BFOR_state = HIGH;
byte BREV_state = HIGH;
byte CFOR_state = HIGH;
byte CREV_state = HIGH;
byte DFOR_state = HIGH;
byte DREV_state = HIGH;

enum ControlLine {
  CTRL_AFOR = 0,
  CTRL_AREV,
  CTRL_BFOR,
  CTRL_BREV,
  CTRL_CFOR,
  CTRL_CREV,
  CTRL_DFOR,
  CTRL_DREV
};

struct ManualControlConfig {
  const ControlLine *lines;
  uint8_t lineCount;
};

const ControlLine forwardControlLines[] = {CTRL_AFOR, CTRL_BFOR, CTRL_CFOR, CTRL_DFOR};
const ControlLine reverseControlLines[] = {CTRL_AREV, CTRL_BREV, CTRL_CREV, CTRL_DREV};
const ControlLine leftControlLines[] = {CTRL_AREV, CTRL_BREV, CTRL_CFOR, CTRL_DFOR};
const ControlLine rightControlLines[] = {CTRL_AFOR, CTRL_BFOR, CTRL_CREV, CTRL_DREV};
const ControlLine cornerLeftControlLines[] = {CTRL_BFOR, CTRL_CFOR};
const ControlLine cornerRightControlLines[] = {CTRL_AFOR, CTRL_DFOR};
const ControlLine cornerBottomLeftControlLines[] = {CTRL_BREV, CTRL_CREV};
const ControlLine cornerBottomRightControlLines[] = {CTRL_AREV, CTRL_DREV};
const ControlLine neutralControlLines[] = {};

const char *movementLabels[] = {
  "forward",
  "reverse",
  "left",
  "right",
  "cornerLeft",
  "cornerRight",
  "cornerBottomLeft",
  "cornerBottomRight",
  "neutral"
};

const ManualControlConfig manualControlProfiles[] = {
  {forwardControlLines, 4},
  {reverseControlLines, 4},
  {leftControlLines, 4},
  {rightControlLines, 4},
  {cornerLeftControlLines, 2},
  {cornerRightControlLines, 2},
  {cornerBottomLeftControlLines, 2},
  {cornerBottomRightControlLines, 2},
  {neutralControlLines, 0}
};


/***CODE************************************************/
void setup()
{
  pinMode(ANALOG_X_PIN, INPUT);
  pinMode(ANALOG_Y_PIN, INPUT);
  pinMode(ANALOG_X2_PIN, INPUT);
  pinMode(ANALOG_Y2_PIN, INPUT);
  //pinMode(ANALOG_BUTTON_PIN, INPUT_PULLUP);

 pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
 pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  
  pinMode( SV1READ, INPUT );
  pinMode( SV2READ, INPUT );
  setAllEngr100ControlStatesReleased();
  lastPrintTime = millis();                       // start log display timer
  sizeof_p_data = sizeof( struct payload_data );
  sizeof_c_data = sizeof( struct control_data );
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  lcd1.begin();
  lcd1.clear();
  lcd1.print("SystemReady");

  delay(1000);
  initializeRF24();
   
  delay(1000);

  if(SD.begin(chipSelect))
  {
    SDPresent = true;
    if( diagnostics ) Serial.println("SDCard found, opening file for writing");
    setupDataFile();
  }
  else
  {
    if ( diagnostics ) Serial.println("No SD card present - continue without");
    SDPresent = false;
  }

  if( diagnostics )
  {
    printf_begin();                       // print rf24 diagnositcs on start-up
    radio.printDetails();
  }
  
  radio.writeAckPayload(1, &c_data, sizeof_c_data );  // preloadACK buffer
  radio.startListening();                             // listen for payload!!
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
  radioCheckAndReply();
  refreshControlData();
  processPayloadData();
}

/*********************************************************
*    FUNCTIONS
*********************************************************/
void initializeRF24()
{
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);           //power options LOW, HIGH, MAX
  radio.setDataRate(RF24_250KBPS);         //rate options to 2Mbps
  radio.enableAckPayload();                 //enable slave reply
  uint8_t channel = ( payloadNumber -1 ) * 7 + 10;     //space channels between payloads
  radio.setChannel( channel );
  //radio.setRetries(10, 10);               //master side only; retry time&number, time = 250us+Nx250us, 
  for ( int i = 0; i < 5; i++ ) nodeAddress[i] = channel;      //create node address from payld#
  radio.openReadingPipe(1, nodeAddress);    //nodeAddress much match at each end
  //delay(1000);
}

/*************************************************************/
void radioCheckAndReply(void)
/* send Ack on receipt of msg from master
 */
{
  while(!radio.available())  delay(1);       // just wait, nothing else to do
  radio.read( &p_data, sizeof_p_data );      // send ACK data
  if( diagnostics )
  {
    Serial.println("Received data from master - sending response data.");
    serialPrintBuf( (uint16_t *)&p_data, sizeof_p_data/2 );
  }
}

/*************************************************************/
void refreshControlData()    //generate new control data
{
  readControlSettings();
  radio.writeAckPayload(1, &c_data, sizeof_c_data );
}

byte readAnalogAxisLevel(int pin)
{
  return map(analogRead(pin), 0, 1023, 0, 255);
}

bool isAnalogButtonPressed(int pin)
{
  return digitalRead(pin) == 0;
}

void setAllEngr100ControlStatesReleased()
{
  AFOR_state = HIGH;
  AREV_state = HIGH;
  BFOR_state = HIGH;
  BREV_state = HIGH;
  CFOR_state = HIGH;
  CREV_state = HIGH;
  DFOR_state = HIGH;
  DREV_state = HIGH;
}

void pressEngr100ControlState(ControlLine line)
{
  switch (line)
  {
    case CTRL_AFOR: AFOR_state = LOW; break;
    case CTRL_AREV: AREV_state = LOW; break;
    case CTRL_BFOR: BFOR_state = LOW; break;
    case CTRL_BREV: BREV_state = LOW; break;
    case CTRL_CFOR: CFOR_state = LOW; break;
    case CTRL_CREV: CREV_state = LOW; break;
    case CTRL_DFOR: DFOR_state = LOW; break;
    case CTRL_DREV: DREV_state = LOW; break;
  }
}

void activateManualControlProfile(const ManualControlConfig &config)
{
  for (uint8_t i = 0; i < config.lineCount; i++)
  {
    pressEngr100ControlState(config.lines[i]);
  }
}

void outputMovementToEngr100ControlStates(const String &movement)
{
  setAllEngr100ControlStatesReleased();

  const uint8_t movementCount = sizeof(movementLabels) / sizeof(movementLabels[0]);
  uint8_t selectedIndex = movementCount - 1;

  for (uint8_t i = 0; i < movementCount; i++)
  {
    if (movement == movementLabels[i])
    {
      selectedIndex = i;
      break;
    }
  }

  activateManualControlProfile(manualControlProfiles[selectedIndex]);
}

void syncVirtualControlStatesToCData()
{
  if (AFOR_state == LOW && AREV_state == HIGH) c_data.m1 = 1;
  else if (AFOR_state == HIGH && AREV_state == LOW) c_data.m1 = 2;
  else c_data.m1 = 0;

  if (BFOR_state == LOW && BREV_state == HIGH) c_data.m2 = 1;
  else if (BFOR_state == HIGH && BREV_state == LOW) c_data.m2 = 2;
  else c_data.m2 = 0;

  if (CFOR_state == LOW && CREV_state == HIGH) c_data.m3 = 1;
  else if (CFOR_state == HIGH && CREV_state == LOW) c_data.m3 = 2;
  else c_data.m3 = 0;

  if (DFOR_state == LOW && DREV_state == HIGH) c_data.m4 = 1;
  else if (DFOR_state == HIGH && DREV_state == LOW) c_data.m4 = 2;
  else c_data.m4 = 0;
}

void interpretation(short x1, short y1, byte pressed1, short x2, short y2, byte pressed2)
{
  #define forwardYPosLimit 32
  #define forwardYNegLimit -16
  #define forwardXPosLimit 129
  #define forwardXNegLimit 5

  #define leftYPosLimit 129
  #define leftXPosLimit 36
  #define leftXNegLimit -28
  #define leftYNegLimit 5

  #define rightYPosLimit -5
  #define rightYNegLimit -129
  #define rightXPosLimit 58
  #define rightXNegLimit -16

  #define downYPosLimit 80
  #define downYNegLimit -3
  #define downXPosLimit -5
  #define downXNegLimit -129

  String movement = "neutral";

  if (x1 > 10 && y1 < -10) {
    movement = "cornerRight";
  }
  else if (x1 < -10 && y1 < -10) {
    movement = "cornerLeft";
  }
  else if (x1 < -10 && y1 > 10) {
    movement = "cornerBottomLeft";
  }
  else if (x1 > 10 && y1 > 10) {
    movement = "cornerBottomRight";
  }
  else if (y1 > forwardYNegLimit && y1 < forwardYPosLimit && x1 < forwardXPosLimit && x1 > forwardXNegLimit) {
    movement = "forward";
  }
  else if (y1 > leftYNegLimit && y1 < leftYPosLimit && x1 < leftXPosLimit && x1 > leftXNegLimit) {
    movement = "left";
  }
  else if (y1 > rightYNegLimit && y1 < rightYPosLimit && x1 < rightXPosLimit && x1 > rightXNegLimit) {
    movement = "right";
  }
  else if (y1 > downYNegLimit && y1 < downYPosLimit && x1 < downXPosLimit && x1 > downXNegLimit) {
    movement = "reverse";
  }

  outputMovementToEngr100ControlStates(movement);

  if (diagnostics)
  {
    Serial.println(movement);
    Serial.println(" ");
    Serial.print("y1:");
    Serial.println(y1);
    Serial.print("x1:");
    Serial.println(x1);
    Serial.println(" ");
  }

  (void)pressed1;
  (void)x2;
  (void)y2;
  (void)pressed2;
}

/************************************************************/
void readControlSettings()
/* e100-600 ROV DESIGNERS NOTE!
* This function is where you customize your thruster controls.
* The default program reads the position of 8 switches connected
* to the controller arduino at pins 2 -9, one each
* for foward and reverse for 4 thrusters and then sets the value
* of c_data.m1 -> c_data.m4 to either a 0(off),1(forward) or 
* 2(reverse). As an ROV system designer you can come up with any 
* thruster control hardware or programming for your shore controller
* that you like, using pins 2-9 for input and as long
* as in the end, you set each of c_data.m(1-4) to
* a 0,1 or 2 each time through the loop.
*/ 
{
  analog analog1;
  analog analog2;

  analog1.x = readAnalogAxisLevel(ANALOG_X_PIN) - ANALOG_X_CORRECTION;
  analog1.y = readAnalogAxisLevel(ANALOG_Y_PIN) - ANALOG_Y_CORRECTION;
  analog1.button.pressed = 0; //isAnalogButtonPressed(ANALOG_BUTTON_PIN);

  analog2.x = readAnalogAxisLevel(ANALOG_X2_PIN) - ANALOG_X2_CORRECTION;
  analog2.y = readAnalogAxisLevel(ANALOG_Y2_PIN) - ANALOG_Y2_CORRECTION;
  analog2.button.pressed = 0;

  interpretation(analog1.x, analog1.y, analog1.button.pressed,
                 analog2.x, analog2.y, analog2.button.pressed);

  syncVirtualControlStatesToCData();

  // set servos if in use.
  if( usingServo1 ) c_data.s1angle = map( analogRead(SV1READ),0,1023,0,180 );
  else c_data.s1angle = 90;

  if( usingServo2 ) c_data.s2angle = constrain(map(analog2.y, -128, 127, 0, 180), 45, 120);
  else c_data.s2angle = 90;
  
  if( diagnostics )
  {
    Serial.print("Servo 1 is set to: ");
    Serial.println( c_data.s1angle );
  }
  
  if( diagnostics ) 
  {
    Serial.println("Control data buffer: ");
    serialPrintBuf( cdwordptr, sizeof_c_data/2 );
  }
}

/*********************************************************/
void processPayloadData()
/* log data if SD present and Print to LCD on logging Interval
 */
{
  //print_p_data();            //diagnositc
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
    lcd1.cursorLeft();
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
    //lcd1.print( String( p_data.badPktCnt) );
    lcd1.cursorLeft();
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
    Serial.println("Data in the buffer is: ");
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
    if( diagnostics ) Serial.println("write header to file");
  }
  else
  {
    if( diagnostics ) Serial.println("SD failed to open - abort SD use");
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
    dataline += String( 90 - (int)p_data.pitch );     // recast to signed
    dataline += ",";
    dataline += String( 180 - (int)p_data.roll );     // recast to signed
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
  if( diagnostics ) Serial.print("Data written to SDcard\n");
  return;
}

/*********************************************************/
