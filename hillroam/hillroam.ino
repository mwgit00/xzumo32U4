// Pololu Zumo 32U4 Arduino Sketch
//
// This program tries to keep the robot roaming around on a hilly surface.
// If it starts pointing downhill, it will try to back up and rotate
// to point uphill before roaming again.

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4Motors motors;

LSM303 compass;


#define K_DELAY 50  // sensor loop delay

const uint8_t K_MODE_IDLE = 0;
const uint8_t K_MODE_TEST = 1;
const uint8_t K_MODE_ROAM = 2;
uint8_t MODE = K_MODE_IDLE;

char report[120];



//---------------------------------------------------
// ACCELEROMETER CALIBRATION

#define K_CALIB_CT  80
#define K_CALIB_DIV (K_CALIB_CT / 7)  // display 7 dots (+1 final dot)
int16_t acc_x_level = 0;
int16_t acc_y_level = 0;
int16_t acc_z_level = 0;

void calib()
{
  int16_t n = 1;
  lcd.clear();
  lcd.print("Calib.");  
  lcd.gotoXY(0, 1);
  delay(500);  

  while (n < K_CALIB_CT)
  {
    // incrementally update average accelerations to get "flat" base values
    // large deviations from this average will indicate a tilt
    compass.read(); 
    acc_x_level += (compass.a.x - acc_x_level) / n;
    acc_y_level += (compass.a.y - acc_y_level) / n;
    acc_z_level += (compass.a.z - acc_z_level) / n;
    n++;
#if 0 // test code
    snprintf_P(report, sizeof(report),
      PSTR("A: %6d %6d %6d"),
      acc_x_level, acc_y_level, acc_z_level);
    Serial.println(report);
#endif
    delay(K_DELAY);
    
    // update progress display
    lcd.gotoXY(n / K_CALIB_DIV, 1);  
    lcd.print(".");
  }

  lcd.clear();
  lcd.print("Ready!");
  lcd.gotoXY(0, 1);  
  lcd.print("Press A.");
  buttonA.waitForButton();
}



//---------------------------------------------------
// TEST MODE

int8_t test_acc_justify(int16_t i)
{
  int8_t w = 3;
  int16_t abs_i = abs(i);
  if (abs_i >= 10) w--;
  if (abs_i >= 100) w--;
  if (i < 0) w--;
  return w;
}

void test_acc()
{
  lcd.clear();
  lcd.print("  AX  AY");
  while (1)
  {
    compass.read();
    int16_t acc_x = compass.a.x - acc_x_level;
    int16_t acc_y = compass.a.y - acc_y_level;
#if 0
    // full accelerometer data can be sent to serial monitor  
    int16_t acc_z = compass.a.z - acc_z_level;
    snprintf_P(report, sizeof(report),
      PSTR("A: %6d %6d %6d"),
      acc_x, acc_y, acc_z);
    Serial.println(report);
#endif
    acc_x = constrain(acc_x / 100, -99, 99);
    acc_y = constrain(acc_y / 100, -99, 99);
    lcd.gotoXY(0, 1);
    lcd.print("        ");
    lcd.gotoXY(0 + test_acc_justify(acc_x), 1);
    lcd.print(acc_x);
    lcd.gotoXY(4 + test_acc_justify(acc_y), 1);
    lcd.print(acc_y);
    delay(100);    
  }
}



//---------------------------------------------------
// MOTOR RAMPING

#define K_RAMP_SAMPLES 11

// motors go in short bursts (ramps)
// acceleration scale factors ramp the velocity
// scale factor is 0-1 with a 10x multiplier in table
int16_t k_acc_fac[K_RAMP_SAMPLES] = {0,3,7,10,7,3,0,0,0,0,0};
uint8_t k_ramp = 0;
int16_t k_toggle = 0;

void ramp_motors(int16_t vmax)
{
  // k_toggle is 1 or 0 so one motor is always off
  // then apply velocity ramp factors
  int16_t v1 = (1 - k_toggle) * vmax;
  int16_t v2 = k_toggle * vmax;
  int16_t fac = k_acc_fac[k_ramp];
  motors.setSpeeds((v1 * fac) / 10, (v2 * fac) / 10);

  // toggle the active motor when count rolls over
  k_ramp++;
  if (k_ramp == K_RAMP_SAMPLES)
  {
    k_ramp = 0;
    k_toggle = 1 - k_toggle;  // toggle
  }
}



// return true if expr is true for max counts
bool is_ct_reached(bool expr, uint8_t* pct, uint8_t max)
{
  bool result = false;
  if (expr)
  {
    (*pct)++;
    if (*pct >= max)
    {
      result = true;
    }
  }
  else
  {
    *pct = 0;
  }
  return result;
}



//---------------------------------------------------
// ROAM MODE

const uint8_t E_FLAT = 0;
const uint8_t E_DOWN = 1;
const uint8_t E_SPIN = 2;

const int16_t K_ACC_PT_DOWN = -3000;
const uint8_t K_FLAT_CT = 3;
const uint8_t K_DOWN_CT = 3;
const uint8_t K_SPIN_CT = 3;

void roam()
{
  uint8_t state = E_FLAT;
  int16_t vmax = 200;
  int16_t vspin = 200;
  uint8_t ct = 0;
  bool flag = false;

  lcd.clear();
  lcd.print("Roaming!");
  ledGreen(1);
  delay(500);

  while (1)
  {
    // this paces everything
    delay(K_DELAY);
    
    // read sensors and apply calibration offsets
    // then update median filters
    compass.read();
    int16_t acc_x = compass.a.x - acc_x_level;
    int16_t acc_y = compass.a.y - acc_y_level;

    switch (state)
    {
      case E_FLAT:
        // normal flat motion moving one motor at a time
        ramp_motors(vmax);
        flag = (acc_x < K_ACC_PT_DOWN);
        if (is_ct_reached(flag, &ct, K_FLAT_CT))
        {
          // robot started pointing downhill
          ledGreen(0);
          ledRed(1);
          state = E_DOWN;
          ct = 0;
        }
        break;
      case E_DOWN:
        // continue ramping in reverse direction
        // to undo whatever motion made robot point downhill
        ramp_motors(-vmax);
        flag = (acc_x > -1200);
        if (is_ct_reached(flag, &ct, K_DOWN_CT) && (k_ramp == 0))
        {
          // robot has stopped its ramp motion
          // and is not pointing downhill as much
          // so spin to try to point uphill again
          ledRed(0);
          ledYellow(1);
          delay(500);
          // best direction for spin determined experimentally
          if (k_toggle == 0)
          {
            vspin = vmax;
          }
          else
          {
            vspin = -vmax;
          }
          // start spinning
          // the small delay afterwards may inhibit
          // any accelerometer jumps caused by start of spin
          motors.setSpeeds(vspin/2, -vspin/2);
          delay(250);
          state = E_SPIN;
          ct = 0;
        }
        break;
      case E_SPIN:
        // keep spinning until pointing uphill again
        // TODO -- or iteration limit is reached ?
        flag = ((acc_y < 500) && (acc_y > -500) && (acc_x > 0));
        if (is_ct_reached(flag, &ct, K_SPIN_CT))
        {
          ledYellow(0);
          ledGreen(1);
          motors.setSpeeds(0, 0);
          // let everything settle before resuming roam
          delay(500);
          state = E_FLAT;
          ct = 0;
        }
        break;
      default:
        break;
    }
  }
}



//---------------------------------------------------
// MAIN ARDUINO FUNCTIONS

void setup()
{
  uint8_t ct_a = 0;
  uint8_t ct_b = 0;
  
  Wire.begin();
  ledYellow(0);
  ledGreen(0);
  ledRed(0);
  lcd.clear();

  if (!compass.init())
  {
    // red light to let user know compass init failed
    ledRed(1);
    while(1)
    {
      lcd.print("COMP ERR");
      delay(500);
    }
  }

  compass.enableDefault();

  // display app title and wait for button press
  // prompt for A but B will enter "secret" test mode
  lcd.clear();
  lcd.print("Hillroam");
  lcd.gotoXY(0, 1);  
  lcd.print("Press A.");
  while (1)
  {
    delay(10);
    if (buttonA.getSingleDebouncedPress())
    {
      while (!buttonA.getSingleDebouncedRelease())
      {
        delay(10);
      }
      MODE = K_MODE_ROAM;
      break;
    }
    if (buttonB.getSingleDebouncedPress())
    {
      ledYellow(1);
      while (!buttonB.getSingleDebouncedRelease())
      {
        delay(10);
      }
      MODE = K_MODE_TEST;
      break;
    }
  }
}

void loop()
{
  // always calibrate accelerometers
  // then enter either test mode or normal roaming mode
  calib();
  if (MODE == K_MODE_TEST)
  {
    test_acc();
  }
  else
  {
    roam();
  }
}

