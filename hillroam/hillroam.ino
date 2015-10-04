// Pololu Zumo 32U4 Arduino Sketch
//
// (adapted from InertialSensors example)
//
// This program tries to keep the robot roaming around on a hilly surface.
// If it starts pointing downhill, it will try to back up and rotate
// to point uphill before roaming again.

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;

LSM303 compass;


#define K_DELAY 50  // sensor loop delay


char report[120];

int16_t acc_x_level = 0;
int16_t acc_y_level = 0;
int16_t acc_z_level = 0;

# define MAX_CT 3
int16_t acc_x_max[MAX_CT];
uint8_t acc_x_max_i = 0;
int16_t acc_y_max[MAX_CT];
uint8_t acc_y_max_i = 0;
int16_t acc_z_max[MAX_CT];
uint8_t acc_z_max_i = 0;



void buff_flush(int16_t* arr)
{
  uint8_t i = 0;
  while (i < MAX_CT)
  {
    arr[i] = 0;
    i++;
  }
}

int16_t buff_update_med(int16_t* arr, uint8_t* arr_i, int16_t a)
{
  // insert in buffer
  arr[acc_z_max_i] = a;
  (*arr_i)++;
  if (*arr_i == MAX_CT)
  {
    *arr_i = 0;
  }
  
  // find max and min of buffer
  int16_t amax = -32768;
  int16_t amin = 32767;
  uint8_t i = 0;
  while (i < MAX_CT)
  {
    if (arr[i] >= amax)
    {
      amax = arr[i];
    }
    if (arr[i] <= amin)
    {
      amin = arr[i];
    }
    i++;
  }
  return amax + amin;
}



#define K_CALIB_CT  80
#define K_CALIB_DIV (K_CALIB_CT / 7)  // display 7 dots (+1 final dot)

void calib()
{
  int16_t n = 1;
  lcd.clear();
  lcd.print("Calib.");  
  lcd.gotoXY(0, 1);  

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



void test_acc()
{
  lcd.clear();
  lcd.print("TestAcc!");

  buff_flush(acc_x_max);
  buff_flush(acc_y_max);
  buff_flush(acc_z_max);

  while (1)
  {
    compass.read();
    int16_t diff_x = compass.a.x - acc_x_level;
    int16_t diff_y = compass.a.y - acc_y_level;
    int16_t diff_z = compass.a.z - acc_z_level;
    int16_t med_x = buff_update_med(acc_x_max, &acc_x_max_i, diff_x);
    int16_t med_y = buff_update_med(acc_y_max, &acc_y_max_i, diff_y);
    int16_t med_z = buff_update_med(acc_z_max, &acc_z_max_i, diff_z);
    
    snprintf_P(report, sizeof(report),
      PSTR("A: %6d %6d %6d"),
      med_x, med_y, med_z);
    Serial.println(report);
    delay(100);    
  }
}



#define K_ROAM_RATE 9

// acceleration scale factors for ramping velocity
// 0, 0, 0.33*Vel, 0.5*Vel, Vel, 0.5*Vel, 0.33*Vel, 0, 0
int16_t k_acc_fac[K_ROAM_RATE] = {500,500,3,2,1,2,3,500,500};

void roam()
{
  lcd.clear();
  lcd.print("Roaming!");
  delay(500);

  int16_t vtog = 100;
  int16_t v = 100;
  int16_t v1 = v + vtog;
  int16_t v2 = v - vtog;
  uint8_t k = 0;
  
  ledGreen(1);

  while (1)
  {
    // read sensors and apply calibration offsets
    // then update median filters
    compass.read();
    int16_t diff_x = compass.a.x - acc_x_level;
    int16_t diff_y = compass.a.y - acc_y_level;
    int16_t med_x = buff_update_med(acc_x_max, &acc_x_max_i, diff_x);
    int16_t med_y = buff_update_med(acc_y_max, &acc_y_max_i, diff_y);

    if (med_x < -2000)
    {
      ledGreen(0);
      ledRed(1);
      
      // robot started going downhill
      // so immediately go into reverse to decrease X tilt
      // keep going in reverse until tilt is reduced
      // or iteration limit is reached
      motors.setSpeeds(-v1 / 2, -v2 / 2);
      int8_t n = 100;
      while (n--)
      {
        delay(K_DELAY);
        compass.read();
        diff_x = compass.a.x - acc_x_level;
        diff_y = compass.a.y - acc_y_level;
        med_x = buff_update_med(acc_x_max, &acc_x_max_i, diff_x);
        med_y = buff_update_med(acc_y_max, &acc_y_max_i, diff_y);
        if (med_x > -1500)
        {
          break;
        }
      }

      ledRed(0);
      ledYellow(1);
      
      // now spin until pointing uphill again
      // or iteration limit is reached      
      motors.setSpeeds(-vtog, vtog);
      n = 100;
      while (n--)
      {
        delay(K_DELAY);
        compass.read();
        diff_x = compass.a.x - acc_x_level;
        diff_y = compass.a.y - acc_y_level;
        med_x = buff_update_med(acc_x_max, &acc_x_max_i, diff_x);
        med_y = buff_update_med(acc_y_max, &acc_y_max_i, diff_y);
        if ((med_y < 500) && (med_y > -500) && (med_x > 0))
        {
          break;
        }
      }

      ledYellow(0);
      ledGreen(1);
      
      // this will restart toggling logic below...
      k = K_ROAM_RATE;
    }

    // toggle the motor speeds
    // and apply velocity ramp factors
    if (k >= K_ROAM_RATE)
    {
      k = 0;
      vtog = -vtog;
      v1 = v + vtog;
      v2 = v - vtog;
    }
    motors.setSpeeds(v1 / k_acc_fac[k], v2 / k_acc_fac[k]);
    k++;
    
    delay(K_DELAY);
  }
}



void setup()
{
  Wire.begin();
  ledYellow(0);
  ledGreen(0);
  ledRed(0);

  if (!compass.init())
  {
    // red light to let user know compass init failed
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to detect the compass."));
      delay(100);
    }
  }

  compass.enableDefault();

  // display app title and wait for button press
  lcd.clear();
  lcd.print("Hillroam");
  lcd.gotoXY(0, 1);  
  lcd.print("Press A.");
  buttonA.waitForButton();
  delay(500);  
}



void loop()
{
  calib();
  //test_acc();
  roam();
}

