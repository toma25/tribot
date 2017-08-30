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
#define TURN_DURATION     250  // ms

#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

// A sensors reading must be greater than or equal to this
// threshold in order for the program to consider that sensor as
// seeing an object.
const uint8_t sensorThreshold = 1;

// The maximum speed to drive the motors while turning.  400 is
// full speed.
const uint16_t turnSpeedMax = 400;

// The minimum speed to drive the motors while turning.  400 is
// full speed.
const uint16_t turnSpeedMin = 200;

const uint16_t forwardSpeed = 300;
const uint16_t maxForwardSpeed = 400;

const uint16_t reverseSpeed = 200;

// The amount to decrease the motor speed by during each cycle
// when an object is seen.
const uint16_t deceleration = 5;

// The amount to increase the speed by during each cycle when an
// object is not seen.
const uint16_t acceleration = 10;

#define LEFT 0
#define RIGHT 1

// Stores the last indication from the sensors about what
// direction to turn to face the object.  When no object is seen,
// this variable helps us make a good guess about which direction
// to turn.
bool senseDir = RIGHT;

bool charge = false;

// True if the robot is turning left (counter-clockwise).
bool turningLeft = false;

// True if the robot is turning right (clockwise).
bool turningRight = false;

// If the robot is turning, this is the speed it will use.
uint16_t turnSpeed = turnSpeedMax;

// The time, in milliseconds, when an object was last seen.
uint16_t lastTimeObjectSeen = 0;

void setup()
{
  proxSensors.initFrontSensor();

  // Wait for the user to press A before driving the motors.
  lcd.clear();
  for (int i = 0; i < 50; i++) {
    ledYellow(i % 2);
  }
  lcd.print(F("Press A"));
  buttonA.waitForButton();
  lineSensors.initThreeSensors();
  lcd.clear();
}

void turnRight()
{
  motors.setSpeeds(turnSpeed, -turnSpeed);
  turningLeft = false;
  turningRight = true;
}

void reverse()
{
  motors.setSpeeds(-reverseSpeed, -reverseSpeed);
  turningLeft = false;
  turningRight = false;
}

void driveForward(int speed)
{
  motors.setSpeeds(speed, speed);
  turningLeft = false;
  turningRight = false;
}

void turnLeft()
{
  motors.setSpeeds(-turnSpeed, turnSpeed);
  turningLeft = true;
  turningRight = false;
}

void stop()
{
  motors.setSpeeds(0, 0);
  turningLeft = false;
  turningRight = false;
}

void loop()
{
  ledRed(charge);

  lineSensors.read(lineSensorValues);

  if (lineSensorValues[0] < QTR_THRESHOLD)
  {
    // If leftmost sensor detects line, reverse and turn to the
    // right.
    motors.setSpeeds(-reverseSpeed, -reverseSpeed);
    delay(REVERSE_DURATION);
    motors.setSpeeds(turnSpeedMax, -turnSpeedMax);
    delay(TURN_DURATION);
    driveForward(forwardSpeed);
    charge = false;
  }
  else if (lineSensorValues[NUM_SENSORS - 1] < QTR_THRESHOLD)
  {
    // If rightmost sensor detects line, reverse and turn to the
    // left.
    motors.setSpeeds(-reverseSpeed, -reverseSpeed);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-turnSpeedMax, turnSpeedMax);
    delay(TURN_DURATION);
    driveForward(forwardSpeed);
    charge = false;
  }
  else
  {
    // Read the front proximity sensor and gets its left value (the
    // amount of reflectance detected while using the left LEDs)
    // and right value.
    proxSensors.read();
    uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
    uint8_t rightValue = proxSensors.countsFrontWithRightLeds();

    // Determine if an object is visible or not.
    bool objectSeen = leftValue >= sensorThreshold || rightValue >= sensorThreshold;

    if (objectSeen)
    {
      // An object is visible, so we will start decelerating in
      // order to help the robot find the object without
      // overshooting or oscillating.
      turnSpeed -= deceleration;
    }
    else
    {
      // An object is not visible, so we will accelerate in order
      // to help find the object sooner.
      turnSpeed += acceleration;
    }

    // Constrain the turn speed so it is between turnSpeedMin and
    // turnSpeedMax.
    turnSpeed = constrain(turnSpeed, turnSpeedMin, turnSpeedMax);

    if (objectSeen)
    {
      // An object seen.
      ledYellow(1);

      lastTimeObjectSeen = millis();

      bool lastTurnRight = turnRight;

      if (leftValue > 4 && rightValue > 4)
      {
        charge = true;
        driveForward(maxForwardSpeed);
      }
      else if (leftValue < rightValue)
      {
        // The right value is greater, so the object is probably
        // closer to the robot's right LEDs, which means the robot
        // is not facing it directly.  Turn to the right to try to
        // make it more even.
        turnRight();
        senseDir = RIGHT;
      }
      else if (leftValue > rightValue)
      {
        // The left value is greater, so turn to the left.
        turnLeft();
        senseDir = LEFT;
      }
      else
      {
        // The values are equal
        driveForward(maxForwardSpeed);
      }
    }
    else
    {
      // No object is seen, so just keep turning in the direction
      // that we last sensed the object.
      ledYellow(0);

      if (senseDir == RIGHT)
      {
        turnRight();
      }
      else
      {
        turnLeft();
      }
    }

    lcd.gotoXY(0, 0);
    lcd.print(leftValue);
    lcd.print(' ');
    lcd.print(rightValue);
    lcd.gotoXY(0, 1);
    lcd.print(turningRight ? 'R' : (turningLeft ? 'L' : ' '));
    lcd.print(' ');
    lcd.print(turnSpeed);
    lcd.print(' ');
    lcd.print(' ');
  }
}
