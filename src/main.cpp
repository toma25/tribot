/* This example uses the front proximity sensor on the Zumo 32U4
Front Sensor Array to locate an opponent robot or any other
reflective object. Using the motors to turn, it scans its
surroundings. If it senses an object, it turns on its yellow LED
and attempts to face towards that object. */

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4Buzzer buzzer;



// This might need to be tuned for different lighting conditions,
// surfaces, etc.
#define QTR_THRESHOLD     1000  // microseconds

// These might need to be tuned for different motor types.
#define REVERSE_DURATION  200  // ms
#define TURN_DURATION     300  // ms
#define STATE_SEARCH_RIGHT 0
#define STATE_SEARCH_LEFT 1
#define STATE_DESTROY 2

#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

const uint8_t sensorThreshold = 3; // How far we detect an object (higher = closer)

const uint16_t highSweepSpeed = 400;
const uint16_t lowSweepSpeed = 150;
const uint16_t normalSpeed = 400;
const uint16_t attackSpeed = 400; // Ojbect is seen, full speed?
const uint16_t slowerAttackSpeed = attackSpeed * 0.4; // If we see the object at an angle, speed of slower wheel

const uint16_t timeInState = 600;

const uint16_t turnSpeedMax = 400; // How fast we turn

const uint16_t reverseSpeed = normalSpeed;

#define LEFT 0
#define RIGHT 1


long startTimeForState = millis();
int state = STATE_SEARCH_LEFT;

void driveForward(uint16_t speed) {
  motors.setSpeeds(speed, speed);
}

void sweepLeft(){
  motors.setSpeeds(lowSweepSpeed, highSweepSpeed);
}

void sweepRight(){
  motors.setSpeeds(highSweepSpeed, lowSweepSpeed);
}


void reverse(uint16_t speed) {
  motors.setSpeeds(-speed, -speed);
}

void stop() {
  motors.setSpeeds(0, 0);
}


void rotate(int dir) {
  if (dir == LEFT) {
    motors.setSpeeds(normalSpeed, -normalSpeed);
  } else {
    motors.setSpeeds(-normalSpeed, normalSpeed);
  }
}

void changeState(int newState) {
  if (newState != state) {
      state = newState;
      startTimeForState = millis();
  }
}

long getTimeInState () {
  return millis() - startTimeForState;
}

void setup() {
  proxSensors.initFrontSensor();

  // Wait for the user to press A before driving the motors.
  lcd.clear();
  lcd.print(F("Press A"));
  buttonA.waitForButton();
  long now = millis();
  long startTime = now + 3000;

  while (startTime > millis()){
    buzzer.play(">g32>>c32");
    delay(10);
  }

  lineSensors.initThreeSensors();
  lcd.clear();
}

void destroy(uint8_t leftValue, uint8_t rightValue) {
  changeState(STATE_DESTROY);
  lcd.clear();
  ledYellow(false);
  ledRed(true);
  lcd.gotoXY(0, 1);
  lcd.print(leftValue + " " + rightValue);
  lcd.gotoXY(0, 0);
  if (leftValue > rightValue) {
    //lcd.print("SL"); // Slight LEFT
    motors.setLeftSpeed(slowerAttackSpeed);
    motors.setRightSpeed(attackSpeed);
  } else if (leftValue < rightValue) {
    //lcd.print("SR"); // Slight RIGHT
    motors.setLeftSpeed(attackSpeed);
    motors.setRightSpeed(slowerAttackSpeed);
  } else {
    //lcd.print("SF"); // Straight FORWARD
    motors.setLeftSpeed(attackSpeed);
    motors.setRightSpeed(attackSpeed);
  }
}

void search (uint8_t leftValue, uint8_t rightValue) {
  if (state == STATE_SEARCH_LEFT) {
      sweepLeft();
      if (getTimeInState() > timeInState) {
        changeState(STATE_SEARCH_RIGHT);
      }
  } else if (state == STATE_SEARCH_RIGHT) {
      sweepRight();
      if (getTimeInState() > timeInState) {
        changeState(STATE_SEARCH_LEFT);
      }
  } else {
    sweepRight();
    changeState(STATE_SEARCH_RIGHT);
  }
  ledYellow(true);
  ledRed(false);
}

void lineDetection() {
  lcd.clear();
  lineSensors.read(lineSensorValues);

  if (lineSensorValues[0] < QTR_THRESHOLD)
  {
    // If leftmost sensor detects line, reverse and turn to the
    // right.
    lcd.gotoXY(0, 0);
    lcd.print("LINE LEFT");
    reverse(reverseSpeed);
    delay(REVERSE_DURATION);
    motors.setSpeeds(turnSpeedMax, -turnSpeedMax);
    delay(TURN_DURATION);
    driveForward(normalSpeed);
    delay(REVERSE_DURATION);
    stop();
    changeState(STATE_SEARCH_RIGHT);
  }
  else if (lineSensorValues[NUM_SENSORS - 1] < QTR_THRESHOLD)
  {
    // If rightmost sensor detects line, reverse and turn to the
    // left.
    lcd.gotoXY(0, 0);
    lcd.print("LINE RIGHT");
    reverse(reverseSpeed);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-turnSpeedMax, turnSpeedMax);
    delay(TURN_DURATION);
    driveForward(normalSpeed);
    delay(REVERSE_DURATION);
    stop();
    changeState(STATE_SEARCH_LEFT);
  }

}

void loop() {
  lineDetection();

  proxSensors.read();
  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();

  bool objectSeen = leftValue >= sensorThreshold || rightValue >= sensorThreshold;

  if (objectSeen || state == STATE_DESTROY) {
    destroy(leftValue, rightValue);
  } else {
    search(leftValue, rightValue);
  }
}
