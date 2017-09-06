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


// This might need to be tuned for different lighting conditions,
// surfaces, etc.
#define QTR_THRESHOLD     1000  // microseconds

// These might need to be tuned for different motor types.
#define REVERSE_DURATION  200  // ms
#define TURN_DURATION     800  // ms
#define SEARCH 0
#define DESTROY 1

#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

const uint8_t sensorThreshold = 3; // How far we detect an object (higher = closer)

const uint16_t normalSpeed = 150;
const uint16_t attackSpeed = 300; // Ojbect is seen, full speed?
const uint16_t slowerAttackSpeed = attackSpeed / 2; // If we see the object at an angle, speed of slower wheel

const uint16_t turnSpeedMax = 200; // How fast we turn

const uint16_t reverseSpeed = normalSpeed;

#define LEFT 0
#define RIGHT 1

int state = SEARCH;

void driveForward(uint16_t speed) {
  motors.setSpeeds(speed, speed);
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

void setup() {
  proxSensors.initFrontSensor();

  // Wait for the user to press A before driving the motors.
  lcd.clear();
  lcd.print(F("Press A"));
  buttonA.waitForButton();
  lineSensors.initThreeSensors();
  lcd.clear();
}

void lineDetection() {
  lcd.clear();
  lineSensors.read(lineSensorValues);

  if (lineSensorValues[0] < QTR_THRESHOLD)
  {
    // If leftmost sensor detects line, reverse and turn to the
    // right.
    reverse(reverseSpeed);
    delay(REVERSE_DURATION);
    motors.setSpeeds(turnSpeedMax, -turnSpeedMax);
    delay(TURN_DURATION);
    driveForward(normalSpeed);
    delay(REVERSE_DURATION);
    stop();
    lcd.gotoXY(0, 0);
    lcd.print("LINE");
  }
  else if (lineSensorValues[NUM_SENSORS - 1] < QTR_THRESHOLD)
  {
    // If rightmost sensor detects line, reverse and turn to the
    // left.
    reverse(reverseSpeed);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-turnSpeedMax, turnSpeedMax);
    delay(TURN_DURATION);
    driveForward(normalSpeed);
    delay(REVERSE_DURATION);
    stop();
    lcd.gotoXY(0, 0);
    lcd.print("LINE");
  }
  state = SEARCH;
}

void loop() {
  lineDetection();

  proxSensors.read();
  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();

  bool objectSeen = leftValue >= sensorThreshold || rightValue >= sensorThreshold;

  if (objectSeen) {
    state = DESTROY;
  } else {
    state = SEARCH;
  }

  switch (state) {
    case SEARCH:
      motors.setLeftSpeed(100);
      motors.setRightSpeed(150);
      ledYellow(true);
      ledRed(false);
      break;
    case DESTROY:
      lcd.clear();
      ledYellow(false);
      ledRed(true);
      lcd.gotoXY(0, 1);
      lcd.print(leftValue + " " + rightValue);
      lcd.gotoXY(0, 0);
      if (leftValue > rightValue) {
        lcd.print("SL"); // Slight LEFT
        motors.setLeftSpeed(slowerAttackSpeed);
        motors.setRightSpeed(attackSpeed);
      } else if (leftValue < rightValue) {
        lcd.print("SR"); // Slight RIGHT
        motors.setLeftSpeed(attackSpeed);
        motors.setRightSpeed(slowerAttackSpeed);
      } else {
        lcd.print("SF"); // Straight FORWARD
        motors.setLeftSpeed(attackSpeed);
        motors.setRightSpeed(attackSpeed);
      }
      break;
  }
}
