// Pololu Zumo 32U4 Arduino Sketch
//
// (adapted from InertialSensors example)

/* This example reads the raw values from the
LSM303D accelerometer on the Zumo 32U4, and
prints those raw values to the serial monitor.

The accelerometer readings can be converted to units of g using
the conversion factors specified in the "Sensor characteristics"
table in the LSM303 datasheet.  The default full-scale (FS)
setting is +/- 2 g, so the conversion factor is 0.61 mg/LSB
(least-significant bit).  A raw reading of 16384 would correspond
to 1 g.

*/

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;

LSM303 compass;



char report[120];

int16_t acc_x_level = 0;
int16_t acc_y_level = 0;
int16_t acc_z_level = 0;

# define MAX_CT 7
int16_t acc_z_max[MAX_CT];
uint8_t acc_z_max_i = 0;



void flush(int16_t* arr)
{
  uint8_t i = 0;
  while (i < MAX_CT)
  {
    arr[i] = 0;
    i++;
  }
}

void update_max_min(int16_t* arr, uint8_t* arr_i, int16_t a, int16_t* pmax, int16_t* pmin)
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
  *pmax = amax;
  *pmin = amin;
}



#define K_CALIB_CT  60
#define K_CALIB_DIV (K_CALIB_CT / 7)  // display 7 dots (+1 final dot)

void calib()
{
  int16_t n = 1;
  lcd.clear();
  lcd.print("Calib.");  
  lcd.gotoXY(0, 1);  

  while (n < K_CALIB_CT)
  {
    // incrementally update average of Z accel. to get "flat" base value
    // large deviations from this average will indicate a tilt
    compass.read(); 
    acc_x_level += (compass.a.x - acc_x_level) / n;
    acc_y_level += (compass.a.y - acc_y_level) / n;
    acc_z_level += (compass.a.z - acc_z_level) / n;
    n++;
    //snprintf_P(report, sizeof(report),
    //  PSTR("A: %6d %6d %6d"),
    //  acc_x_level, acc_y_level, acc_z_level);
    //Serial.println(report);
    delay(100);
    
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
  
  while (1)
  {
    compass.read();
    int16_t diff_x = compass.a.x - acc_x_level;
    int16_t diff_y = compass.a.y - acc_y_level;
    // invert z acc so a "tilt" is a positive value
    int16_t diff_z = acc_z_level - compass.a.z;
    int16_t max_z;
    int16_t min_z;
    update_max_min(acc_z_max, &acc_z_max_i, diff_z, &max_z, &min_z);
    
    snprintf_P(report, sizeof(report),
      PSTR("A: %6d %6d %6d"),
      diff_x, diff_y, max_z + min_z);
    Serial.println(report);
    delay(100);    
  }
}



// TODO -- make this work
void roam()
{
  lcd.clear();
  lcd.print("Roaming!");
  delay(500);

  motors.setSpeeds(75, 75);
  
  while (1)
  {
    // read sensors and apply calibration offsets
    // invert z acc so a "tilt" is a positive value
    compass.read();
    int16_t diff_x = compass.a.x - acc_x_level;
    int16_t diff_y = compass.a.y - acc_y_level;
    int16_t diff_z = acc_z_level - compass.a.z;

    // median filter for z
    // try to smooth out the "tilt" signal
    int16_t max_z;
    int16_t min_z;
    update_max_min(acc_z_max, &acc_z_max_i, diff_z, &max_z, &min_z);

    //snprintf_P(report, sizeof(report),
    //  PSTR("A: %6d %6d %6d"),
    //  diff_x, diff_y, max_z + min_z);
    //Serial.println(report);

    if (max_z + min_z > 1000)
    {
      ledYellow(1);
      
      // slam on the brakes
      motors.setSpeeds(-100, -100);
      delay(100);
      motors.setSpeeds(0, 0);
      
      buttonA.waitForButton();
      delay(500);

      flush(acc_z_max);
      motors.setSpeeds(75, 75);
    }
    else
    {
      ledYellow(0);
    }

    delay(100);
  }
}



void setup()
{
  Wire.begin();

  if (!compass.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to detect the compass."));
      delay(100);
    }
  }

  compass.enableDefault();

  // wait for the user to press A
  lcd.clear();
  lcd.print("Hillroam");
  lcd.gotoXY(0, 1);  
  lcd.print("Press A.");
  buttonA.waitForButton();
}



void loop()
{
  calib();
  //test_acc();
  roam();
}

