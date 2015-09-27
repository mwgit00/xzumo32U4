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

LSM303 compass;
L3G gyro;

char report[120];

int32_t z_acc_level = 0;



#define K_CALIB_CT  100
#define K_CALIB_DIV (K_CALIB_CT / 7)  // display 7 dots (+1 final dot)

void calib()
{
  int32_t n = 1;
  lcd.clear();
  lcd.print("Calib.");  
  lcd.gotoXY(0, 1);  

  while (n < K_CALIB_CT)
  {
    // incrementally update average of Z accel. to get "flat" base value
    // large deviations from this average will indicate a tilt
    compass.read(); 
    z_acc_level = (compass.a.z + z_acc_level * (n - 1)) / n;
    n++;
    //snprintf_P(report, sizeof(report),
    //  PSTR("A: %6d %6d %6d"),
    //  z_acc_level, n, compass.a.z);
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



void roam()
{
  lcd.clear();
  lcd.print("Roaming!");
  
  while (1)
  {
    compass.read();
    int32_t z_diff = z_acc_level - compass.a.z;
    snprintf_P(report, sizeof(report),
      PSTR("A: %6d %6d %6d"),
      compass.a.x, compass.a.y, z_diff);
    Serial.println(report);

    if (z_diff > 200)
    {
      ledYellow(1);
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
  roam();
}

