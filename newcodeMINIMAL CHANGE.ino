/* 
* Last Rev 1/19/2024:  Target: Arduino Nano
* Minimal-change joystick version of Shore Control Programming for ROV V4.
*
* This sketch keeps the original shoreCode structure as much as possible.
* Only the hardware-specific parts were changed so it matches the Nano
* joystick controller used in this repo:
 * - nRF24L01+ on CE=8, CSN=9
 * - Joystick 1 on A2/A3 for thrust direction
 * - Joystick 2 on A6/A7 for servo control
 * - Joystick button on D2
 * - SD uses D10 as chip select so it does not conflict with the joystick pins
*
*/

/****** Set Payload ID# *******/
const uint8_t payloadNumber = 2;        // integer from 1 through 12

/********** Tunables**************************/
#define SERIAL_BAUD 115200
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
#define CE_PIN 8                    // Nano nRF24 CE
#define CSN_PIN 9                   // Nano nRF24 CSN
#define LCD_ADDR 0x27

// This matches the Nano joystick layout used by debuggedNewCode.ino
// and the checked-in wiring diagram.
#define ANALOG_X_PIN A2             // joystick 1 horizontal
#define ANALOG_Y_PIN A3             // joystick 1 vertical
#define ANALOG_BUTTON_PIN 2         // joystick 1 pushbutton
#define ANALOG_X2_PIN A6            // joystick 2 horizontal
#define ANALOG_Y2_PIN A7            // joystick 2 vertical

#define ANALOG_X_CORRECTION 128
#define ANALOG_Y_CORRECTION 128
#define ANALOG_X2_CORRECTION 128
#define ANALOG_Y2_CORRECTION 128

const int chipSelect = 10;          // safe SD CS pin for Nano wiring


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
uint16_t * const pdwordptr = (uint16_t*)&p_data;
uint16_t * const cdwordptr = (uint16_t*)&c_data;
size_t sizeof_p_data;
size_t sizeof_c_data;
long lastPrintTime;
long curPrintTime;
bool SDPresent = false;                               // if no SD just carry-on
int printCounter = 1;                                 // print 2nd or 3rd line?
byte nodeAddress[5] = {};                             // pipe address:set from payload #


/***CODE************************************************/
void setup()
{
  pinMode(ANALOG_X_PIN, INPUT);
  pinMode(ANALOG_Y_PIN, INPUT);
  pinMode(ANALOG_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ANALOG_X2_PIN, INPUT);
  pinMode(ANALOG_Y2_PIN, INPUT);
  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CSN_PIN, HIGH);
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);

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
  radio.enableAckPayload();                //enable slave reply
  uint8_t channel = ( payloadNumber -1 ) * 7 + 10;     //space channels between payloads
  radio.setChannel( channel );
  for ( int i = 0; i < 5; i++ ) nodeAddress[i] = channel;
  radio.openReadingPipe(1, nodeAddress);   //nodeAddress must match at each end
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

/************************************************************/
void readControlSettings()
/* Minimal change from shoreCode:
 * instead of reading 8 digital thruster switches, read joystick values
 * and write the final 0/1/2 motor commands directly into c_data.
 */
{
  short x1 = readCenteredAxis(ANALOG_X_PIN, ANALOG_X_CORRECTION);
  short y1 = readCenteredAxis(ANALOG_Y_PIN, ANALOG_Y_CORRECTION);
  short x2 = readCenteredAxis(ANALOG_X2_PIN, ANALOG_X2_CORRECTION);
  short y2 = readCenteredAxis(ANALOG_Y2_PIN, ANALOG_Y2_CORRECTION);
  const char *movement = "neutral";

  c_data.m1 = 0;
  c_data.m2 = 0;
  c_data.m3 = 0;
  c_data.m4 = 0;

  if( x1 > 10 && y1 < -10 )
  {
    c_data.m1 = 1;
    c_data.m4 = 1;
    movement = "cornerRight";
  }
  else if( x1 < -10 && y1 < -10 )
  {
    c_data.m2 = 1;
    c_data.m3 = 1;
    movement = "cornerLeft";
  }
  else if( x1 < -10 && y1 > 10 )
  {
    c_data.m2 = 2;
    c_data.m3 = 2;
    movement = "cornerBottomLeft";
  }
  else if( x1 > 10 && y1 > 10 )
  {
    c_data.m1 = 2;
    c_data.m4 = 2;
    movement = "cornerBottomRight";
  }
  else if( y1 > -16 && y1 < 32 && x1 < 129 && x1 > 5 )
  {
    c_data.m1 = 1;
    c_data.m2 = 1;
    c_data.m3 = 1;
    c_data.m4 = 1;
    movement = "forward";
  }
  else if( y1 > 5 && y1 < 129 && x1 < 36 && x1 > -28 )
  {
    c_data.m1 = 2;
    c_data.m2 = 2;
    c_data.m3 = 1;
    c_data.m4 = 1;
    movement = "left";
  }
  else if( y1 > -129 && y1 < -5 && x1 < 58 && x1 > -16 )
  {
    c_data.m1 = 1;
    c_data.m2 = 1;
    c_data.m3 = 2;
    c_data.m4 = 2;
    movement = "right";
  }
  else if( y1 > -3 && y1 < 80 && x1 < -5 && x1 > -129 )
  {
    c_data.m1 = 2;
    c_data.m2 = 2;
    c_data.m3 = 2;
    c_data.m4 = 2;
    movement = "reverse";
  }

  if( usingServo1 ) c_data.s1angle = constrain(map(x2, -128, 127, 0, 180), 0, 180);
  else c_data.s1angle = 90;

  if( usingServo2 ) c_data.s2angle = constrain(map(y2, -128, 127, 0, 180), 45, 120);
  else c_data.s2angle = 90;
  
  if( diagnostics )
  {
    Serial.println(movement);
    Serial.print("y1:");
    Serial.println(y1);
    Serial.print("x1:");
    Serial.println(x1);
    Serial.print("Servo 1 is set to: ");
    Serial.println( c_data.s1angle );
    Serial.print("Servo 2 is set to: ");
    Serial.println( c_data.s2angle );
    Serial.println("Control data buffer: ");
    serialPrintBuf( cdwordptr, sizeof_c_data/2 );
  }
}

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
    dataline += String( 90 - (int)p_data.pitch );   // recast to signed
    dataline += ",";
    dataline += String( 180 - (int)p_data.roll );   // recast to signed
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
}

/*********************************************************/
