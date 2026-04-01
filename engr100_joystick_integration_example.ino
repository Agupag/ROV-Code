/*
  PART 9 ONLY - VIRTUAL MANUAL CONTROL STATES

  This version does NOT drive physical AFOR/AREV/... pins.
  It sets virtual state variables (AFOR_state, AREV_state, etc.) to HIGH/LOW,
  then converts those states to c_data.m1..m4.

  Why this matches your request:
  - You can edit engr100 manually and keep control logic software-only.
  - No pinMode(AFOR,...), no digitalWrite(AFOR,...), no digitalRead(mPins[i]).
  - Behavior matches manual INPUT_PULLUP semantics:
      HIGH = released, LOW = pressed.
*/

// Keep joystick pin names exactly like joystickCode.ino.
#define ANALOG_X_PIN A1
#define ANALOG_Y_PIN A0
#define ANALOG_BUTTON_PIN 2
#define ANALOG_X2_PIN A2
#define ANALOG_Y2_PIN A3

// Default center corrections from joystickCode.ino.
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

byte readAnalogAxisLevel(int pin)
{
  return map(analogRead(pin), 0, 1023, 0, 255);
}

bool isAnalogButtonPressed(int pin)
{
  return digitalRead(pin) == 0;
}

// Virtual manual control states (not physical pin writes).
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
  // Manual INPUT_PULLUP semantics:
  // forward = LOW/HIGH, reverse = HIGH/LOW, neutral = HIGH/HIGH.

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
    Serial.print(" AFOR="); Serial.print(AFOR_state);
    Serial.print(" AREV="); Serial.print(AREV_state);
    Serial.print(" BFOR="); Serial.print(BFOR_state);
    Serial.print(" BREV="); Serial.print(BREV_state);
    Serial.print(" CFOR="); Serial.print(CFOR_state);
    Serial.print(" CREV="); Serial.print(CREV_state);
    Serial.print(" DFOR="); Serial.print(DFOR_state);
    Serial.print(" DREV="); Serial.println(DREV_state);
  }

  (void)pressed1;
  (void)x2;
  (void)y2;
  (void)pressed2;
}

void readControlSettingsUsingYourJoystickFunctionsAsIs()
{
  analog analog1;
  analog analog2;

  analog1.x = readAnalogAxisLevel(ANALOG_X_PIN) - ANALOG_X_CORRECTION;
  analog1.y = readAnalogAxisLevel(ANALOG_Y_PIN) - ANALOG_Y_CORRECTION;
  analog1.button.pressed = isAnalogButtonPressed(ANALOG_BUTTON_PIN);

  analog2.x = readAnalogAxisLevel(ANALOG_X2_PIN) - ANALOG_X2_CORRECTION;
  analog2.y = readAnalogAxisLevel(ANALOG_Y2_PIN) - ANALOG_Y2_CORRECTION;
  analog2.button.pressed = 0;

  interpretation(analog1.x, analog1.y, analog1.button.pressed,
                 analog2.x, analog2.y, analog2.button.pressed);

  // Convert virtual AFOR/AREV/... states into c_data motor commands.
  syncVirtualControlStatesToCData();

  if (usingServo1) c_data.s1angle = map(analogRead(SV1READ), 0, 1023, 0, 180);
  else c_data.s1angle = 90;

  if (usingServo2) c_data.s2angle = map(analogRead(SV2READ), 0, 1023, 0, 180);
  else c_data.s2angle = 90;
}

/*
  Paste usage in engr100 code:

  1) Add joystick pin setup in setup():
       pinMode(ANALOG_X_PIN, INPUT);
       pinMode(ANALOG_Y_PIN, INPUT);
       pinMode(ANALOG_X2_PIN, INPUT);
       pinMode(ANALOG_Y2_PIN, INPUT);
       pinMode(ANALOG_BUTTON_PIN, INPUT_PULLUP);

  2) Remove reliance on reading AFOR..DREV physical pins for motor command logic.
     Use virtual state conversion by calling:
       readControlSettingsUsingYourJoystickFunctionsAsIs();

  3) In readControlSettings(), replace body with:
       readControlSettingsUsingYourJoystickFunctionsAsIs();

  Note:
  - AFOR/AREV/etc in engr100 are pin-number constants. They cannot be assigned.
  - Use AFOR_state/AREV_state/... as the assignable HIGH/LOW variables.
*/
