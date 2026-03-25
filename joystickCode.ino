#define ANALOG_X_PIN A1 
#define ANALOG_Y_PIN A0 
#define ANALOG_BUTTON_PIN 2 

#define ANALOG_X2_PIN A2
#define ANALOG_Y2_PIN A3
	 
//Default values when axis not actioned 
#define ANALOG_X_CORRECTION 128 
#define ANALOG_Y_CORRECTION 128 
#define ANALOG_X2_CORRECTION 128
#define ANALOG_Y2_CORRECTION 128
	 
struct button { 
	 byte pressed = 0; 
}; 
	 
struct analog { 
	 short x, y; 
	 button button; 
}; 
	 
void setup() 
{ 
	 pinMode(ANALOG_BUTTON_PIN, INPUT_PULLUP); 
	 Serial.begin(115200); 
} 
	 
void loop() 
{ 
	 analog analog1; 
	 analog analog2; 
	 
	 analog1.x = readAnalogAxisLevel(ANALOG_X_PIN) - ANALOG_X_CORRECTION; 
	 analog1.y = readAnalogAxisLevel(ANALOG_Y_PIN) - ANALOG_Y_CORRECTION; 
	 analog1.button.pressed = isAnalogButtonPressed(ANALOG_BUTTON_PIN); 

	 analog2.x = readAnalogAxisLevel(ANALOG_X2_PIN) - ANALOG_X2_CORRECTION; 
	 analog2.y = readAnalogAxisLevel(ANALOG_Y2_PIN) - ANALOG_Y2_CORRECTION; 
	 analog2.button.pressed = 0;   // no second button assigned
	 
	 drawDualAsciiController(
	   analog1.x, analog1.y, analog1.button.pressed,
	   analog2.x, analog2.y, analog2.button.pressed
	 );

	 delay(200); 
} 
	 
byte readAnalogAxisLevel(int pin) 
{ 
	 return map(analogRead(pin), 0, 1023, 0, 255); 
} 
	 
bool isAnalogButtonPressed(int pin) 
{ 
	 return digitalRead(pin) == 0; 
} 




void interpretation(short x1, short y1, byte pressed1, short x2, short y2, byte pressed2)
{
  //these are the values that determine each section of the joystick. The joystick is mapped in x and y values, ranging from -128 to 128. 0 is the center.
  //In order to figure out what section the joystick is currently in, we need to set limits for each section, then apply them through if statements to find where the joystick is.
  
  #define forwardYUpLimit 128
  #define forwardXRightLimit 64
  #define forwardXLeftLimit -64
  #define forwardYDownLimit 120
  String movement = "neutral";
/*

    ( | ) is what is being targeted by these limits in the x range
    ( [] ) is the edge of the joystick (the upper and lower bounds that it can track)
    ( _ ) is the y range targeting of the limits
    ( = ) is the y limits that the joystick can track (edge)
    ( x ) is where the code knows that the joystick is in the (go forward) position
    The image below shows what would happen with the limits above. It targets "forward" which is very forward and not too much left or right. Later, we can make the limits a diagonal. but for now it is boxes.



========================================================= y=128(edge)
    []           |  x     x  x |              []
    []           |x   x   x x  |              []
    []           |   x  x   x  |              []
    []           | x   x x    x|              []
    []           |  x   x   x  |              []
    []_ _ _ _ __________|______________ _ _ _ [] _ y=120
    []         x=-64   x=0    x=64            []
    []                                        []
  x=-128(edge)                            x=128(edge)


============================================================ y=0(center)

*/



// NOW WE NEED TO USE IF STATEMENTS TO FIND OUT WHICH SECTION IT IS IN

//forward movement check from earlier
if (y1 > forwardYDownLimit && y1 < forwardYUpLimit && x1 < forwardXRightLimit && x1 > forwardXLeftLimit){
  movement = "forward";
}



//placeholder movement if statements

if (y1 > ____YDownLimit && y1 < _____YUpLimit && x1 < ____XRightLimit && x1 > _____XLeftLimit){
  movement = "reverse";
}

if (y1 > ____YDownLimit && y1 < _____YUpLimit && x1 < ____XRightLimit && x1 > _____XLeftLimit){
  movement = "diagonalLeft";
}

if (y1 > ____YDownLimit && y1 < _____YUpLimit && x1 < ____XRightLimit && x1 > _____XLeftLimit){
  movement = "diagonalRight";
}

if (y1 > ____YDownLimit && y1 < _____YUpLimit && x1 < ____XRightLimit && x1 > _____XLeftLimit){
  movement = "Right";
}

if (y1 > ____YDownLimit && y1 < _____YUpLimit && x1 < ____XRightLimit && x1 > _____XLeftLimit){
  movement = "Left";
}

if (y1 > ____YDownLimit && y1 < _____YUpLimit && x1 < ____XRightLimit && x1 > _____XLeftLimit){
  movement = "_____";
}

if (y1 > ____YDownLimit && y1 < _____YUpLimit && x1 < ____XRightLimit && x1 > _____XLeftLimit){
  movement = "diagonalBackLeft";
}

if (y1 > ____YDownLimit && y1 < _____YUpLimit && x1 < ____XRightLimit && x1 > _____XLeftLimit){
  movement = "diagonalBackRight";
}

}




// ==========. THEN WE OUTPUT THE MOVEMENT TO THE PINS. HERE IS EXAMPLE CODE    ===========

if (movement == "forward"){
  //pinout code for all 3 thrusters on
}

////////////////////////////////////////////////////////////////



//BELOW THIS IS VIBECODED UNUSED CODE ==============================================================================================================


void drawDualAsciiController(short x1, short y1, byte pressed1, short x2, short y2, byte pressed2)
{
  const int W = 21;
  const int H = 11;

  int px1 = map(x1, -128, 128, 0, W - 1);
  int py1 = map(y1, 128, -128, 0, H - 1);

  int px2 = map(x2, -128, 128, 0, W - 1);
  int py2 = map(y2, 128, -128, 0, H - 1);

  Serial.println();
  Serial.println(F("      JOYSTICK 1                     JOYSTICK 2"));
  Serial.println(F("         Y+                            Y+"));
  Serial.println(F("          ^                             ^"));

  for (int row = 0; row < H; row++) {
    Serial.print(F("     |"));
    for (int col = 0; col < W; col++) {
      if (col == px1 && row == py1) {
        Serial.print('O');
      } else if (col == W / 2 && row == H / 2) {
        Serial.print('+');
      } else if (col == W / 2) {
        Serial.print('|');
      } else if (row == H / 2) {
        Serial.print('-');
      } else {
        Serial.print(' ');
      }
    }

    Serial.print(F("|     "));

    Serial.print(F("|"));
    for (int col = 0; col < W; col++) {
      if (col == px2 && row == py2) {
        Serial.print('O');
      } else if (col == W / 2 && row == H / 2) {
        Serial.print('+');
      } else if (col == W / 2) {
        Serial.print('|');
      } else if (row == H / 2) {
        Serial.print('-');
      } else {
        Serial.print(' ');
      }
    }
    Serial.println(F("|"));
  }

  Serial.println(F("          +-------> X+                  +-------> X+"));

  Serial.print(F("J1 X = "));
  Serial.print(x1);
  Serial.print(F("   Y = "));
  Serial.print(y1);
  Serial.print(F("   BTN = "));
  Serial.print(pressed1 ? "PRESSED" : "NOT PRESSED");

  Serial.print(F("      J2 X = "));
  Serial.print(x2);
  Serial.print(F("   Y = "));
  Serial.print(y2);
  Serial.print(F("   BTN = "));
  Serial.println(pressed2 ? "PRESSED" : "NOT PRESSED");
}