// Pololu Zumo 32U4 Arduino Sketch
//
// (adapted from FaceTowardsOpponent example)
//
// This program uses a PID loop to "balance" the prox sensor
// reading so the robot spins to face an object in front of it.

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA;


#define K_V_MAX 400 // maximum motor velocity

// these were tweaked for a stock assembled Zumo with 75:1 gear ratio
#define K_P 100
#define K_I 20
#define K_D 200

#define BUFF_SIZE 8
#define BUFF_SIZE_MASK  (BUFF_SIZE - 1)



// A sensor's reading must be greater than or equal to this
// threshold in order for the program to consider that sensor as
// seeing an object.
const uint8_t sensorThreshold = 1;



// the following code chunk implements a buffer
// which can provide a rolling average
// (size must be a power of 2)

int8_t buff[BUFF_SIZE];
uint8_t buff_ix = 0;
int8_t buff_sum = 0;

int16_t error;
int16_t e_diff;
int16_t error_prev;
int16_t pid;

void buff_update(int8_t x)
{
  buff_sum -= buff[buff_ix];
  buff[buff_ix] = x;
  buff_sum += x;
  buff_ix = (buff_ix + 1) & BUFF_SIZE_MASK;
}

void buff_flush()
{
  uint8_t i = BUFF_SIZE;
  buff_sum = 0;
  while (i)
  {
    i--;
    buff[i] = 0;
  }
}



void setup()
{
  proxSensors.initFrontSensor();
  buff_flush();

  // Wait for the user to press A before driving the motors.
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("Spintrak");
  lcd.gotoXY(0, 1);
  lcd.print(F("Press A."));
  buttonA.waitForButton();
  lcd.clear();
}



void spin(int16_t v)
{
  motors.setSpeeds(-v, v);
}



void loop()
{
  // Read the front proximity sensor and gets its left value (the
  // amount of reflectance detected while using the left LEDs)
  // and right value.
  proxSensors.read();
  int8_t proxL = proxSensors.countsFrontWithLeftLeds();
  int8_t proxR = proxSensors.countsFrontWithRightLeds();

  // Determine if an object is visible or not.
  bool is_object_seen = (proxL >= sensorThreshold) || (proxR >= sensorThreshold);

  if (is_object_seen)
  {
    // indicate with LED and update PID loop
    ledYellow(1);
    error = proxL - proxR;
    buff_update(error);
    e_diff = error - error_prev;
    pid = K_P * error + K_I * buff_sum + K_D * e_diff;
    pid = constrain(pid, -K_V_MAX, K_V_MAX);
    spin(pid);
    error_prev = error;
  }
  else
  {
    // no object is seen so flush error states and stop motors
    // (another option is just to keep turning in the direction
    // that we last sensed the object)
    buff_flush();
    error_prev = 0;
    ledYellow(0);
    spin(0);
  }

  lcd.gotoXY(0, 0);
  lcd.print(proxL);
  lcd.print(' ');
  lcd.print(proxR);
  lcd.gotoXY(0, 1);
  lcd.print(buff_sum);
  lcd.print(' ');
}

