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

#define testPin 10 
#define pinCornerLeft 5
#define pinCornerBottomLeft 4
#define pinCornerBottomRight 7
#define pinCornerRight 6

	 
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
   pinMode(testPin, OUTPUT);
   pinMode(11, OUTPUT);
   pinMode(12, OUTPUT);
   pinMode(13, OUTPUT);
   pinMode(pinCornerRight, OUTPUT);
   pinMode(pinCornerLeft, OUTPUT);
   pinMode(pinCornerBottomRight, OUTPUT);
   pinMode(pinCornerBottomLeft, OUTPUT);
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
	 
	//  drawDualAsciiController(
	//    analog1.x, analog1.y, analog1.button.pressed,
	//    analog2.x, analog2.y, analog2.button.pressed
	//  );
   
  interpretation(analog1.x, analog1.y, analog1.button.pressed, analog2.x, analog2.y, analog2.button.pressed);
	 delay(10); 
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
  

  // //forward
  #define forwardYPosLimit 32
  #define forwardYNegLimit -16
  #define forwardXPosLimit 129
  #define forwardXNegLimit 5



  //   //left
  #define leftYPosLimit 129
  #define leftXPosLimit 36
  #define leftXNegLimit -28
  #define leftYNegLimit 5

  //     //right
  #define rightYPosLimit -5
  #define rightYNegLimit -129
  #define rightXPosLimit 58
  #define rightXNegLimit -16


  //   // //down
  #define downYPosLimit 80
  #define downYNegLimit -3
  #define downXPosLimit -5
  #define downXNegLimit -129




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

// Prioritize corners first, then directions, so only one movement is set
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
  movement = "down";
}
Serial.println(movement);
Serial.println(" ");
Serial.print("y1:");
Serial.println(y1);
Serial.print("x1:");
Serial.println(x1);
Serial.println(" ");




//placeholder movement if statements
/*
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
*/



// ==========. THEN WE OUTPUT THE MOVEMENT TO THE PINS. HERE IS EXAMPLE CODE    ===========

if (movement == "forward"){
  digitalWrite(testPin, HIGH);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  digitalWrite(pinCornerRight, LOW);
  digitalWrite(pinCornerLeft, LOW);
  digitalWrite(pinCornerBottomLeft, LOW);
  digitalWrite(pinCornerBottomRight, LOW);
  delay(10);
}
else if(movement == "right"){
  digitalWrite(11, HIGH);
  digitalWrite(testPin, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  digitalWrite(pinCornerRight, LOW);
  digitalWrite(pinCornerLeft, LOW);
  digitalWrite(pinCornerBottomLeft, LOW);
  digitalWrite(pinCornerBottomRight, LOW);
  delay(10);
}
else if(movement == "left"){
  digitalWrite(12, HIGH);
  digitalWrite(testPin, LOW);
  digitalWrite(11, LOW);
  digitalWrite(13, LOW);
  digitalWrite(pinCornerRight, LOW);
  digitalWrite(pinCornerLeft, LOW);
  digitalWrite(pinCornerBottomLeft, LOW);
  digitalWrite(pinCornerBottomRight, LOW);
  delay(10);
}
else if(movement == "down"){
  digitalWrite(13, HIGH);
  digitalWrite(testPin, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(pinCornerRight, LOW);
  digitalWrite(pinCornerLeft, LOW);
  digitalWrite(pinCornerBottomLeft, LOW);
  digitalWrite(pinCornerBottomRight, LOW);
  delay(10);
}
else if(movement == "cornerRight"){
  digitalWrite(pinCornerRight, HIGH);
  digitalWrite(testPin, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  digitalWrite(pinCornerLeft, LOW);
  digitalWrite(pinCornerBottomLeft, LOW);
  digitalWrite(pinCornerBottomRight, LOW);
  delay(10);
}
else if(movement == "cornerLeft"){
  digitalWrite(pinCornerLeft, HIGH);
  digitalWrite(testPin, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  digitalWrite(pinCornerRight, LOW);
  digitalWrite(pinCornerBottomLeft, LOW);
  digitalWrite(pinCornerBottomRight, LOW);
  delay(10);
}
else if(movement == "cornerBottomLeft"){
  digitalWrite(pinCornerBottomLeft, HIGH);
  digitalWrite(testPin, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  digitalWrite(pinCornerRight, LOW);
  digitalWrite(pinCornerLeft, LOW);
  digitalWrite(pinCornerBottomRight, LOW);
  delay(10);
}
else if(movement == "cornerBottomRight"){
  digitalWrite(pinCornerBottomRight, HIGH);
  digitalWrite(testPin, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  digitalWrite(pinCornerRight, LOW);
  digitalWrite(pinCornerLeft, LOW);
  digitalWrite(pinCornerBottomLeft, LOW);
  delay(10);
}
else{
  digitalWrite(testPin, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  digitalWrite(pinCornerRight, LOW);
  digitalWrite(pinCornerLeft, LOW);
  digitalWrite(pinCornerBottomLeft, LOW);
  digitalWrite(pinCornerBottomRight, LOW);
  delay(10);
}
}




if movement = forward{

  thruster1_forward = on
  thruster2_forward = on
  thruster3_forward = on

}
if movement = left{

  pin1 = forward_thruster_starboard
  pin2 = forward_thruster_port
  pin3 = forward_thruster_mid
  pin4 = forward_thruster_vertical
  pin5 = reverse_thruster_starboard
  pin6 = reverse_thruster_port
  pin7 = reverse_thruster_mid
  pin8 = reverse_thruster_vertical


}

struct thruster{
  forward = {forward_thruster_starboard, forward_thruster_port, forward_thruster_mid};
  reverse = {pin5, pin6, pin7, pin8};
  left_upper_corner = {forward_thruster_port, forward_thruster_mid, reverse_thruster_starboard};
  right_upper_corner = {pin13, pin14, pin15, pin16};
  left_lower_corner = {pin17, pin18, pin19, pin20};
  right_lower_corner = {pin21, pin22, pin23, pin24};
}