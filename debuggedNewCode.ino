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
* data arrival is controlled at the payload. This build displays
* the data on the LCD only. A stopped clock
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
const bool windowIsFront = true;        // set false for reversed payload mount
const long loggingInterval = 1000;      // milliseconds
const int displayFlip = 3;              // number of logs between line 2 flips
const long diagnosticsInterval = 500;   // milliseconds between serial updates
const int joystickDeadzone = 6;         // centered noise band after correction

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


/***********Pin Assignments********************/
// Classic Nano-safe mapping: A6/A7 are analog-only, so use them only for joystick axes.
#define CE_PIN 8                    // chip enable for nRF24L01+
#define CSN_PIN 9                   // chip select for nRF24L01+
#define LCD_ADDR 0x27

// Joystick input pins (from joystickCode.ino)
#define ANALOG_X_PIN A2
#define ANALOG_Y_PIN A3
#define ANALOG_X2_PIN A6
#define ANALOG_Y2_PIN A7

// Default values when axis not actioned
#define ANALOG_X_CORRECTION 128
#define ANALOG_Y_CORRECTION 128
#define ANALOG_X2_CORRECTION 128
#define ANALOG_Y2_CORRECTION 128


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

/************Global Variables******************/
//uint8_t mPins[8] = { AFOR, AREV, BFOR, BREV, CFOR, CREV, DFOR, DREV };
//bool mData[8] = {};
uint16_t * const pdwordptr = (uint16_t*)&p_data;   // byte pointer for struct
uint16_t * const cdwordptr = (uint16_t*)&c_data;
size_t sizeof_p_data;
size_t sizeof_c_data;
long lastPrintTime;
long curPrintTime;
long lastDiagnosticsTime;
bool radioInitialized = false;
bool diagnosticsThisCycle = false;
int printCounter = 1;                               // print 2nd or 3rd line?
byte nodeAddress[5] = {};                  // pipe address:set from payload #

/***CODE************************************************/
void setup()
{
  pinMode(ANALOG_X_PIN, INPUT);
  pinMode(ANALOG_Y_PIN, INPUT);
  pinMode(ANALOG_X2_PIN, INPUT);
  pinMode(ANALOG_Y2_PIN, INPUT);
  pinMode( CSN_PIN, OUTPUT );
  digitalWrite( CSN_PIN, HIGH );
  // Keep the AVR hardware SS pin as output so SPI stays in master mode.
  pinMode( SS, OUTPUT );
  digitalWrite( SS, HIGH );
  lastPrintTime = millis();                       // start log display timer
  lastDiagnosticsTime = 0;
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

  if( diagnostics )
  {
    printf_begin();                       // print rf24 diagnositcs on start-up
    if( radioInitialized ) radio.printDetails();
  }

  if( radioInitialized )
  {
    radio.writeAckPayload(1, &c_data, sizeof_c_data );  // preloadACK buffer
    radio.startListening();                             // listen for payload!!
  }
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
  if( !radioInitialized )
  {
    delay(100);
    return;
  }

  diagnosticsThisCycle = diagnostics && ( millis() - lastDiagnosticsTime >= diagnosticsInterval );
  if( diagnosticsThisCycle )
  {
    lastDiagnosticsTime = millis();
  }

  radioCheckAndReply();
  refreshControlData();
  processPayloadData();
}

/*********************************************************
*    FUNCTIONS
*********************************************************/
void initializeRF24()
{
  radioInitialized = radio.begin();
  if( !radioInitialized )
  {
    if( diagnostics ) Serial.println("RF24 init failed");
    lcd1.clear();
    lcd1.print("RF24 init fail");
    return;
  }

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
  if( diagnosticsThisCycle )
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

short readCenteredAxis(int pin, int correction)
{
  short value = readAnalogAxisLevel(pin) - correction;
  if (abs(value) <= joystickDeadzone) value = 0;
  return value;
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
  short x1 = readCenteredAxis(ANALOG_X_PIN, ANALOG_X_CORRECTION);
  short y1 = readCenteredAxis(ANALOG_Y_PIN, ANALOG_Y_CORRECTION);
  short x2 = readCenteredAxis(ANALOG_X2_PIN, ANALOG_X2_CORRECTION);
  short y2 = readCenteredAxis(ANALOG_Y2_PIN, ANALOG_Y2_CORRECTION);
  const int axisThreshold = 20;
  const char *movement = "neutral";

  c_data.m1 = 0;
  c_data.m2 = 0;
  c_data.m3 = 0;
  c_data.m4 = 0;

  if( y1 < -axisThreshold )
  {
    if( x1 > axisThreshold )
    {
      c_data.m1 = 1;
      c_data.m4 = 1;
      movement = "cornerRight";
    }
    else if( x1 < -axisThreshold )
    {
      c_data.m2 = 1;
      c_data.m3 = 1;
      movement = "cornerLeft";
    }
    else
    {
      c_data.m1 = 1;
      c_data.m2 = 1;
      c_data.m3 = 1;
      c_data.m4 = 1;
      movement = "forward";
    }
  }
  else if( y1 > axisThreshold )
  {
    if( x1 > axisThreshold )
    {
      c_data.m1 = 2;
      c_data.m4 = 2;
      movement = "cornerBottomRight";
    }
    else if( x1 < -axisThreshold )
    {
      c_data.m2 = 2;
      c_data.m3 = 2;
      movement = "cornerBottomLeft";
    }
    else
    {
      c_data.m1 = 2;
      c_data.m2 = 2;
      c_data.m3 = 2;
      c_data.m4 = 2;
      movement = "reverse";
    }
  }
  else if( x1 < -axisThreshold )
  {
    c_data.m1 = 2;
    c_data.m2 = 2;
    c_data.m3 = 1;
    c_data.m4 = 1;
    movement = "left";
  }
  else if( x1 > axisThreshold )
  {
    c_data.m1 = 1;
    c_data.m2 = 1;
    c_data.m3 = 2;
    c_data.m4 = 2;
    movement = "right";
  }

  // Servo 1 follows joystick 2 X, servo 2 follows joystick 2 Y.
  if( usingServo1 ) c_data.s1angle = constrain(map(x2, -128, 127, 0, 180), 0, 180);
  else c_data.s1angle = 90;

  if( usingServo2 ) c_data.s2angle = constrain(map(y2, -128, 127, 0, 180), 45, 120);
  else c_data.s2angle = 90;
  
  if( diagnosticsThisCycle )
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
  }
  
  if( diagnosticsThisCycle ) 
  {
    Serial.print("RF24 thrusters m1=");
    Serial.print(c_data.m1);
    Serial.print(" m2=");
    Serial.print(c_data.m2);
    Serial.print(" m3=");
    Serial.print(c_data.m3);
    Serial.print(" m4=");
    Serial.println(c_data.m4);
    Serial.println("Control data buffer: ");
    serialPrintBuf( cdwordptr, sizeof_c_data/2 );
  }
}

/*********************************************************/
void processPayloadData()
/* Print payload data to LCD on the logging interval.
 */
{
  //print_p_data();            //diagnositc
  curPrintTime = millis();
  if( curPrintTime - lastPrintTime > loggingInterval )
  {
    printoLCD();
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
  lcd1.print( String( p_data.yaw ) );
  lcd1.print( " " );
  if ( windowIsFront )
  {
    lcd1.print( String( ((int)p_data.pitch) - 90 ) );
    lcd1.print( " " );
    lcd1.print( String( ((int)p_data.roll) - 180 ) );
  }
  else
  {
    lcd1.print( String( 90 - (int)p_data.pitch ) );
    lcd1.print( " " );
    lcd1.print( String( 180 - (int)p_data.roll ) );
  }
  lcd1.print( " " );
  lcd1.print( String( p_data.pres ) );
  lcd1.print( "cm" );
  lcd1.setCursor(0,1);
  if( printCounter <= displayFlip )
  {
    lcd1.print( String( p_data.vlt / 100.0, 1 ) );
    lcd1.print( "v " );
    lcd1.print( String( p_data.amp / 100.0, 1 ) );
    lcd1.print( "a " );
    lcd1.print( String( p_data.temp / 100 ) );
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
  if( diagnosticsThisCycle )
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
