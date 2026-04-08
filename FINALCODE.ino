/****** Set Payload ID# *******/
const uint8_t payloadNumber = 2;        // integer from 1 through 12

/********** Tunables**************************/
#define SERIAL_BAUD 115200
const bool windowIsFront = true;        // set false for reversed payload mount
const unsigned long loggingInterval = 1000; // milliseconds
const int displayFlip = 3;              // number of logs between line 2 flips
const String fname = "ROVDATA.CSV";     // SD card file name

/********** ConstantS*************************/

#define T_VERT    m1
#define T_REAR_L  m2
#define T_REAR_R  m3
#define T_FRONT   m4

#define SERVO1_MIN_ANGLE 0
#define SERVO1_MAX_ANGLE 180

/**** Bools *******/
bool diagnostics = false;
bool usingServo1 = true;
bool usingServo2 = false;

/********** Libraries*************************/
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>

/***********Pin Assignments********************/
#define CE_PIN 8
#define CSN_PIN 2
#define LCD_ADDR 0x27

#define ANALOG_X_PIN A2
#define ANALOG_Y_PIN A3
#define ANALOG_X2_PIN A6
#define ANALOG_Y2_PIN A7

#define ANALOG_X_CORRECTION 128
#define ANALOG_Y_CORRECTION 128
#define ANALOG_X2_CORRECTION 128
#define ANALOG_Y2_CORRECTION 128

#define BUTTON_UP_PIN 3
#define BUTTON_DOWN_PIN 4

const int chipSelect = 10;

#define RADIO_TIMEOUT_MS 5000

/************Data Structures******************/
struct payload_data
{
  const uint16_t startmark = 0xFFFF;
  uint16_t badPktCnt;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
  uint16_t vlt;
  uint16_t amp;
  uint16_t pres;
  uint16_t temp;
  uint16_t yr;
  uint16_t mon;
  uint16_t day;
  uint16_t hr;
  uint16_t min;
  uint16_t sec;
  uint16_t tds;
} p_data = {};

struct control_data
{
  const uint16_t startmark = 0xFFFF;
  uint16_t crc;
  uint16_t s1angle = 90;
  uint16_t s2angle = 90;
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} c_data = {};

/*********Objects***************************/
RF24 radio( CE_PIN, CSN_PIN, 2000000 );
LiquidCrystal_I2C lcd1( LCD_ADDR, 16, 2 );
File datafile;

/************Global Variables******************/
uint16_t * const pdwordptr = (uint16_t*)&p_data;
uint16_t * const cdwordptr = (uint16_t*)&c_data;
size_t sizeof_p_data;
size_t sizeof_c_data;
unsigned long lastPrintTime;
unsigned long curPrintTime;
bool SDPresent = false;
int printCounter = 1;
byte nodeAddress[5] = {};

/***CODE************************************************/
void setup()
{
  pinMode(ANALOG_X_PIN, INPUT);
  pinMode(ANALOG_Y_PIN, INPUT);
  pinMode(ANALOG_X2_PIN, INPUT);
  pinMode(ANALOG_Y2_PIN, INPUT);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);

  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CSN_PIN, HIGH);
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);

  lastPrintTime = millis();
  sizeof_p_data = sizeof( struct payload_data );
  sizeof_c_data = sizeof( struct control_data );

  lcd1.init();
  lcd1.backlight();
  lcd1.clear();
  lcd1.print("SystemReady");

  initializeRF24();

  if(SD.begin(chipSelect))
  {
    SDPresent = true;
    setupDataFile();
  }
  else
  {
    SDPresent = false;
  }

  radio.writeAckPayload(1, &c_data, sizeof_c_data);
  radio.startListening();
}

/*********************************************************/
void loop()
{
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
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.enableAckPayload();
  uint8_t channel = ( payloadNumber - 1 ) * 7 + 10;
  radio.setChannel(channel);
  for (int i = 0; i < 5; i++) nodeAddress[i] = channel;
  radio.openReadingPipe(1, nodeAddress);
  return ok;
}

/*************************************************************/
void radioCheckAndReply(void)
{
  unsigned long start = millis();

  while(!radio.available())
  {
    if (millis() - start >= RADIO_TIMEOUT_MS)
    {
      return;
    }
    delay(1);
  }

  radio.read(&p_data, sizeof_p_data);
}

/*************************************************************/
void refreshControlData()
{
  readControlSettings();
  radio.writeAckPayload(1, &c_data, sizeof_c_data);
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
{
  short x1 = readCenteredAxis(ANALOG_X_PIN, ANALOG_X_CORRECTION);
  short y1 = readCenteredAxis(ANALOG_Y_PIN, ANALOG_Y_CORRECTION);
  short y2 = readCenteredAxis(ANALOG_Y2_PIN, ANALOG_Y2_CORRECTION);

  c_data.m1 = 0;
  c_data.m2 = 0;
  c_data.m3 = 0;
  c_data.m4 = 0;

  bool btnUp = (digitalRead(BUTTON_UP_PIN) == LOW);
  bool btnDown = (digitalRead(BUTTON_DOWN_PIN) == LOW);

  if (btnUp && !btnDown)
  {
    c_data.T_VERT = 1;
  }
  else if (btnDown && !btnUp)
  {
    c_data.T_VERT = 2;
  }

  if (y1 < -20)
  {
    if (x1 > 20)
    {
      c_data.T_REAR_L = 1;
      c_data.T_REAR_R = 0;
      c_data.T_FRONT = 1;
    }
    else if (x1 < -20)
    {
      c_data.T_REAR_L = 0;
      c_data.T_REAR_R = 1;
      c_data.T_FRONT = 1;
    }
    else
    {
      c_data.T_REAR_L = 1;
      c_data.T_REAR_R = 1;
      c_data.T_FRONT = 1;
    }
  }
  else if (y1 > 20)
  {
    if (x1 > 20)
    {
      c_data.T_REAR_L = 2;
      c_data.T_REAR_R = 0;
      c_data.T_FRONT = 2;
    }
    else if (x1 < -20)
    {
      c_data.T_REAR_L = 0;
      c_data.T_REAR_R = 2;
      c_data.T_FRONT = 2;
    }
    else
    {
      c_data.T_REAR_L = 2;
      c_data.T_REAR_R = 2;
      c_data.T_FRONT = 2;
    }
  }
  else
  {
    if (x1 > 20)
    {
      c_data.T_REAR_L = 1;
      c_data.T_REAR_R = 2;
      c_data.T_FRONT = 0;
    }
    else if (x1 < -20)
    {
      c_data.T_REAR_L = 2;
      c_data.T_REAR_R = 1;
      c_data.T_FRONT = 0;
    }
  }

  if (usingServo1)
  {
    c_data.s1angle = constrain(map(y2, -128, 127, SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE), SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE);
  }
  else
  {
    c_data.s1angle = 90;
  }

  if (usingServo2)
  {
    c_data.s2angle = constrain(map(y2, -128, 127, 0, 180), 45, 120);
  }
  else
  {
    c_data.s2angle = 90;
  }
}

/*********************************************************/
void processPayloadData()
{
  curPrintTime = millis();
  if (curPrintTime - lastPrintTime > loggingInterval)
  {
    printoLCD();
    if (SDPresent) writeLineToFile();
    lastPrintTime = curPrintTime;
  }
}

/************************************************************/
void printoLCD()
{
  lcd1.clear();
  lcd1.print(String(p_data.yaw));
  lcd1.print(" ");
  if (windowIsFront)
  {
    lcd1.print(String(((int)p_data.pitch) - 90));
    lcd1.print(" ");
    lcd1.print(String(((int)p_data.roll) - 180));
  }
  else
  {
    lcd1.print(String((90 - (int)p_data.pitch)));
    lcd1.print(" ");
    lcd1.print(String((180 - (int)p_data.roll)));
  }
  lcd1.print(" ");
  lcd1.print(String(p_data.pres));
  lcd1.print("cm");
  lcd1.setCursor(0,1);
  if (printCounter <= displayFlip)
  {
    lcd1.print(String((float)(p_data.vlt / 100.0), 1));
    lcd1.print("v ");
    lcd1.print(String((float)(p_data.amp / 100.0), 1));
    lcd1.print("a ");
    lcd1.print(String(p_data.temp / 100));
    lcd1.print("C");
    printCounter++;
  }
  else
  {
    lcd1.print(String(p_data.hr));
    lcd1.print(":");
    lcd1.print(String(p_data.min));
    lcd1.print(":");
    lcd1.print(String(p_data.sec));
    lcd1.print(" ");
    lcd1.print(String(p_data.tds));
    lcd1.print("mho");
    if (++printCounter > 2 * displayFlip) printCounter = 1;
  }
}

/*****************************************************/
void setupDataFile()
{
  if (datafile = SD.open(fname, FILE_WRITE))
  {
    datafile.println("Yaw,Pitch,Roll,mV,mA,pres,degCx10,Yr,Mo,Day,Hr,Min,Sec");
    datafile.flush();
  }
  else
  {
    SDPresent = false;
  }
}

/*********************************************************/
void writeLineToFile()
{
  String dataline = "";
  dataline += String(p_data.yaw);
  dataline += ",";
  if (windowIsFront)
  {
    dataline += String((int)p_data.pitch - 90);
    dataline += ",";
    dataline += String((int)p_data.roll - 180);
  }
  else
  {
    dataline += String(90 - (int)p_data.pitch);
    dataline += ",";
    dataline += String(180 - (int)p_data.roll);
  }
  dataline += ",";
  dataline += String(p_data.vlt * 10);
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
  datafile.println(dataline);
  datafile.flush();
}
