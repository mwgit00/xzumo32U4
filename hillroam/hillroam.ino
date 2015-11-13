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
Zumo32U4Encoders encoders;

LSM303 compass;


#define K_DELAY 50  // sensor loop delay (50ms)

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
// SPEED CONTROL

int16_t pre_err_l = 0;
int16_t pre_err_r = 0;
int16_t enc_l = 0;
int16_t enc_r = 0;
int16_t out_l = 0;
int16_t out_r = 0;

void reset_speed_control()
{
  motors.setSpeeds(0, 0);
  pre_err_l = 0;
  pre_err_r = 0;
  enc_l = 0;
  enc_r = 0;
  out_l = 0;
  out_r = 0;
}

void update_speed_control(int16_t vsetl, int16_t vsetr)
{
  // use Proportional-Derivative control
  // with limits on output
  const int16_t LIMIT = 200;
  const float K_P = 2.0;
  const float K_D = 0.5;

  // get velocity feedback
  enc_l = encoders.getCountsAndResetLeft();
  enc_r = encoders.getCountsAndResetRight();

  // update error terms
  int16_t err_l = vsetl - enc_l;
  int16_t err_r = vsetr - enc_r;
  int16_t d_err_l = err_l - pre_err_l;
  int16_t d_err_r = err_r - pre_err_r;

  // calculate CHANGE to output to correct speed error
  out_l = constrain(out_l + (int)(K_P * err_l + K_D * d_err_l), -LIMIT, LIMIT);
  out_r = constrain(out_r + (int)(K_P * err_r + K_D * d_err_r), -LIMIT, LIMIT);
  motors.setSpeeds(out_l, out_r);

  // update these for calculating error derivative on next pass
  pre_err_l = err_l;
  pre_err_r = err_r;
}



//---------------------------------------------------
// TEST MODE FUNCTIONS

int8_t right_justify_int(int16_t i, int16_t offset)
{
  // right justify 4-digit integer
  // this is default position for single digit
  int8_t w = 3;
  int16_t abs_i = abs(i);
  // scoot back one place if 2 digits
  if (abs_i >= 10) w--;
  // scoot back one place if 3 digits
  if (abs_i >= 100) w--;
  // scoot back one place if negative
  if (i < 0) w--;
  return w + offset;
}

void test_acc()
{
  lcd.clear();
  lcd.print("  AX  AY");
  delay(500);
  
  while (1)
  {
    compass.read();
    int16_t acc_x = compass.a.x - acc_x_level;
    int16_t acc_y = compass.a.y - acc_y_level;
#if 1
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
    lcd.gotoXY(right_justify_int(acc_x, 0), 1);
    lcd.print(acc_x);
    lcd.gotoXY(right_justify_int(acc_y, 4), 1);
    lcd.print(acc_y);
    delay(100);
    if (buttonB.getSingleDebouncedPress())
    {
      break;
    }
  }

  while (!buttonB.getSingleDebouncedRelease())
  {
    // wait for release
  }
}

void test_speed_control()
{
  int16_t vr;
  int16_t vl;
  int16_t v = 20;
  int16_t k = 0;
  
  lcd.clear();
  lcd.print("  VL  VR");
  delay(500);

  while (1)
  {
    // this paces everything
    delay(K_DELAY);

    // toggle speed periodically
    if (k == 0)
    {
      v = -v;
      k = 200;
    }
    k--;
    update_speed_control(v, v);    
    
    lcd.gotoXY(0, 1);
    lcd.print("        ");
    lcd.gotoXY(right_justify_int(enc_l, 0), 1);
    lcd.print(enc_l);
    lcd.gotoXY(right_justify_int(enc_r, 4), 1);
    lcd.print(enc_r);
    if (buttonB.getSingleDebouncedPress())
    {
      break;
    }
  }

  reset_speed_control();
  while (!buttonB.getSingleDebouncedRelease())
  {
    // wait for release
  }
}



//---------------------------------------------------
// UTILITY FUNCTIONS

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
const uint8_t E_REVERSE = 1;
const uint8_t E_SPIN = 2;

const int16_t K_TILT_DOWN = -2400;
const int16_t K_TILT_UP = 2400;
const int16_t K_BALANCE_X = 1200;
const int16_t K_BALANCE_Y = 400;
const uint8_t K_FLAT_CT = 3;
const uint8_t K_DOWN_CT = 3;
const uint8_t K_SPIN_CT = 2;

void roam()
{
  uint8_t state = E_FLAT;
  int16_t vmax = 15;
  int16_t vspin = 0;
  int16_t vl = vmax;
  int16_t vr = vmax;
  
  uint8_t ct = 0;
  uint8_t ct1 = 0;
  uint8_t ct2 = 0;
  bool flag = false;
  bool flag1 = false;
  bool flag2 = false;

  lcd.clear();
  lcd.print("Roaming!");
  ledGreen(1);
  delay(500);

  while (1)
  {
    // this paces everything
    delay(K_DELAY);

    // apply whatever the latest settings are to PID loops
    update_speed_control(vl, vr);
    
    // read sensors and apply calibration offsets
    compass.read();
    int16_t acc_x = compass.a.x - acc_x_level;
    int16_t acc_y = compass.a.y - acc_y_level;

    switch (state)
    {
      case E_FLAT:
        // check if robot started pointing downhill or uphill
        flag1 = is_ct_reached((acc_x < K_TILT_DOWN), &ct1, K_FLAT_CT);
        flag2 = is_ct_reached((acc_x > K_TILT_UP), &ct2, K_FLAT_CT);        
        if (flag1 || flag2)
        {
          vl = -vl;
          vr = -vr;
          state = E_REVERSE;
          ledGreen(0);
          ledRed(1);
          ct1 = 0;
          ct2 = 0;
        }
        break;
      
      case E_REVERSE:
        // check if nearly flat again (balanced in X)
        flag = ((acc_x > -K_BALANCE_X) && (acc_x < K_BALANCE_X));
        if (is_ct_reached(flag, &ct, K_DOWN_CT))
        {
          // robot is not tilting as much
          // so spin to try to point uphill again
          ledRed(0);
          ledYellow(1);
          /*
          if (acc_y < 0)
          {
            vspin = -vmax;
          }
          else
          {
            vspin = vmax;
          }

          // start spinning
          // the small delay afterwards may inhibit
          // any accelerometer jumps caused by start of spin
          vl = vspin;
          vr = -vspin;
          state = E_SPIN;
          */
          vl = -vl;
          vr = -vr;
          state = E_FLAT;
          ct = 0;
        }
        break;
      
      case E_SPIN:
        // keep spinning until Y is nearly balanced
        // and see if tilt on X is reversed
        // TODO -- or iteration limit is reached ?
        flag1 = ((acc_y < K_BALANCE_Y) && (acc_y > -K_BALANCE_Y));
        flag2 = true;//(acc_x > 0);
        flag = flag1 && flag2;
        if (is_ct_reached(flag, &ct, K_SPIN_CT))
        {
          ledYellow(0);
          ledGreen(1);
          vl = vmax;
          vr = vmax;
          state = E_FLAT;
          ct = 0;
        }
        break;
      default:
        break;
    }
  }
}


void cruise()
{
  lcd.clear();
  lcd.print("Cruise!");
  ledGreen(1);
  delay(500);

  int16_t roll = 0;
  int16_t vmax = 25;
  int16_t vl = 0;
  int16_t vr = 0;

  while (1)
  {
    // this paces everything
    delay(K_DELAY);
    
    // read sensors and apply calibration offsets
    // then update median filters
    compass.read();
    int16_t acc_x = compass.a.x - acc_x_level;
    int16_t acc_y = compass.a.y - acc_y_level;

    roll = (int)(acc_y * 0.01);
    vl = constrain(vmax - roll, -vmax, vmax);
    vr = constrain(vmax + roll, -vmax, vmax);
    update_speed_control(vl, vr);
    lcd.gotoXY(0, 1);
    lcd.print("      ");
    lcd.gotoXY(0, 1);
    lcd.print(roll);
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
    while (1)
    {
      // tests will cycle in infinite loop
      // C button will cancel out of each test
      test_acc();
      test_speed_control();
    }
  }
  else
  {
    //roam();
    cruise();
  }
}

