/* -------------------------REFERENCES:---------------------------
  ADXL377 github: https://github.com/sparkfun/ADXL377_Breakout

 ***** Different methods:*****

  1. in depth theory & in depth steps used to calibrate:
  https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/

  2. finding Voltage for x,y,z:

  a:
  https://ez.analog.com/mems/f/q-a/89030/how-to-calibrate-accelerometer-adxl377?ReplySortBy=CreatedDate&ReplySortOrder=Ascending

  b:
  https://www.element14.com/community/community/design-challenges/sudden-impact/blog/2015/05/05/part31--head-impact-detection-adxl377

  3. EQUATION foud here==> accelX_adjusted= ((rawX-zero_g_X)*3.3/1023)/0.0065
  https://forum.arduino.cc/index.php?topic=520719.0

  4.Since 1g isnt enough to calibrate,must create a centrifuge jig to make a much larger G force than 1g:

  https://www.youtube.com/watch?v=RYN-mzIscuQ&t=21s

  5.To drop the accelerometer from a known height and measure the negative acceleration at impact:
  https://forums.adafruit.com/viewtopic.php?f=19&p=752303&fbclid=IwAR2wKB7B49n-Rz2C8WZWP2jweHY9zh97KGPw9wSY28SHVdt4LBs9rHWZEVk

  6.Adafruits Method of Calibration(doesnt work):
  https://learn.adafruit.com/adafruit-analog-accelerometer-breakouts/calibration-and-programming#gravity-as-a-calibration-reference-4-3
  7. using std deviation and error constants from hand calcs
  -------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <Wire.h>
#include <math.h>

//---------------MPU-AXL377 Global Variables------------------------------
//#define Maxcount 200

// Make sure these two variables are correct for your setup
int scale = 200; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377
float accel_volt = 5.0; // assigning accel. input voltage type
float AccelRaw_MAX, AccelRaw_zerog;


unsigned int sumX, sumY, sumZ = 0; // unsigned int, otherwise last 7 digits makes sum <0 , also sum should never be< 0 w/ raw values


int i,  Maxcount = 200, count = 0;
float AvgRawAccelx, AvgRawAccely, AvgRawAccelz;
float deviationX, deviationY, deviationZ, Deviation, DeviationX , DeviationY, DeviationZ, SumDeviationX, SumDeviationY, SumDeviationZ;

float Min_rawY = 512, Max_rawY = 512, Min_rawZ = 512, Max_rawZ = 512, Min_rawX = 512, Max_rawX = 512;


//METHOD: using statistics equations:
float adjusted_gAccel_X, adjusted_gAccel_Y, adjusted_gAccel_Z;
float accelXcount, Xaccel = 0;
char debug_accelDeviation = 'Y';  // debug for calc. average accel. deviation in setup function while loop
unsigned int RawArray_X[200], RawArray_Y[200], RawArray_Z[200];

//--------------------------------------------------------

char debug = 'Y';// 'Y' if debugging and printing all testing values to serial monitor
int t = 0;

void setup() {
  /*
    "analogReference(EXTERNAL)":-------------?

    Tells the arduino that the reference volt value is not 3.3V or 5V, when using an
    external batt source thats not 3.3 or 5v.
  */
  //analogReference(EXTERNAL); // use if power source isnt 3.3 or 5v--------?
  Serial.begin(115200);


  // -------------Assigning sensor type:-------------
  if (accel_volt = 5.0)
  {
    AccelRaw_MAX = 1024;
    AccelRaw_zerog = AccelRaw_MAX / 2;

  }
  else if (accel_volt = 3.3)
  {
    AccelRaw_MAX = 675;
    AccelRaw_zerog = AccelRaw_MAX / 2;

  }
  //--------------Calibrating: --------------
  unsigned int rawX = analogRead(A8);
  unsigned int rawY = analogRead(A7);
  unsigned int  rawZ = analogRead(A9);

  while (count <= Maxcount)
  {

    sumX += rawX;
    sumY += rawY;
    sumZ += rawZ;

    RawArray_X[count] = sumX;
    RawArray_Y[count] = sumY;
    RawArray_Z[count] = sumZ;
    if (debug_accelDeviation == 'Y')
    {
      Serial.print(F("At count: ")); Serial.print(count);
      Serial.print(F("  The raw accel sums(X,Y,Z) are:"));
      Serial.print(RawArray_X[count]);
      Serial.print(",  ");
      Serial.print(RawArray_Y[count]);
      Serial.print(",  ");
      Serial.println(RawArray_Z[count]);
    }

    /*
       The last couple of digits printed reveals that there is something wrong
       with serial or the code... therefore, i will add a condition below, to break the loop when
       the count and sum is at a specific value onwards.

    */

    // The condition, determining which sum gets screwed up 1st(X,Y,or Z):

    if ((RawArray_X[count] < RawArray_X[count - 1]))
    {
      sumX = RawArray_X[count - 1];
      sumY = RawArray_Y[count - 1];
      sumZ = RawArray_Z[count - 1];

      if (debug == 'Y')
      {
        Serial.println(F("Last rawZ accel sums is invalid, at count:"));
        Serial.println(count);
        Serial.println(F("The new sumX is:"));
        Serial.print(sumX);

        Serial.println(F("The new sumY is:"));
        Serial.print(sumY);
        Serial.println(F("The new sumZ is:"));
        Serial.print(sumZ);
      }
      // clear RawArrays to save memory:
      memset(RawArray_X, 0, sizeof RawArray_X);
      memset(RawArray_Y, 0, sizeof RawArray_Y);
      memset(RawArray_Z, 0, sizeof RawArray_Z);
      break;

    }
    if (((RawArray_Y[count] < RawArray_Y[count - 1])))
    {
      sumX = RawArray_X[count - 1];
      sumY = RawArray_Y[count - 1];
      sumZ = RawArray_Z[count - 1];

      if (debug == 'Y')
      {
        Serial.println(F("Last rawZ accel sums is invalid, at count:"));
        Serial.println(count);
        Serial.println(F("The new sumX is:"));
        Serial.print(sumX);

        Serial.println(F("The new sumY is:"));
        Serial.print(sumY);
        Serial.println(F("The new sumZ is:"));
        Serial.print(sumZ);
      }
      // clear RawArrays to save memory:
      memset(RawArray_X, 0, sizeof RawArray_X);
      memset(RawArray_Y, 0, sizeof RawArray_Y);
      memset(RawArray_Z, 0, sizeof RawArray_Z);
      break;
    }

    if (((RawArray_Z[count] < RawArray_Z[count - 1])))
    {
      sumX = RawArray_X[count - 1];
      sumY = RawArray_Y[count - 1];
      sumZ = RawArray_Z[count - 1];

      if (debug == 'Y')
      {
        Serial.println(F("Last rawZ accel sums is invalid, at count:"));
        Serial.println(count);
        Serial.println(F("The new sumX is:"));
        Serial.print(sumX);

        Serial.println(F("The new sumY is:"));
        Serial.print(sumY);
        Serial.println(F("The new sumZ is:"));
        Serial.print(sumZ);
      }
      // clear RawArrays to save memory:
      memset(RawArray_X, 0, sizeof RawArray_X);
      memset(RawArray_Y, 0, sizeof RawArray_Y);
      memset(RawArray_Z, 0, sizeof RawArray_Z);
      break;

    }

    //---------Determining max and min raw accel. values:---------
    if (rawX < Min_rawX)
    {
      Min_rawX = rawX;
    }
    if (rawX > Max_rawX)
    {
      Max_rawX = rawX;
    }
    if (rawY < Min_rawY)
    {
      Min_rawY = rawY;
    }
    if (rawY > Max_rawY)
    {
      Max_rawY = rawY;
    }
    if (rawZ < Min_rawZ)
    {
      Min_rawZ = rawZ;
    }
    if (rawZ > Max_rawZ)
    {
      Max_rawZ = rawZ;
    }
    // applying alternative method==> deviation method:
    float mew1 = AccelRaw_zerog;//,mew2 = sumX/(count+1);
    deviationX += rawX - mew1;
    deviationY += rawY - mew1;
    deviationZ += rawZ - mew1;
    //Serial.println(deviationX);


    count = count + 1;
    delay(100);
  }
  // AVG. DEVIATION:
  DeviationX = deviationX / count; //for accelX
  DeviationY = deviationY / count;
  DeviationZ = deviationZ / count;
  // Get raw accelerometer data for each axis
  if (debug_accelDeviation == 'Y')
  {
    Serial.println("");
    Serial.println(rawX);
    Serial.println(rawY);
    Serial.println(rawZ);



    Serial.print(F("min and max rawX,Y,Z value respectively are:") );
    Serial.print(Min_rawX);
    Serial.print("-" );
    Serial.print(Max_rawX);
    Serial.print(",  ");
    Serial.print(Min_rawY);
    Serial.print("-" );
    Serial.print(Max_rawY);
    Serial.print(",  ");
    Serial.print(Min_rawZ);
    Serial.print("-" );
    Serial.println(Max_rawZ);
  }



}


void loop() {
  // raw Accel is an unsigned int to make sure values received are from 0-AccelRaw_MAX
  unsigned int rawX = analogRead(A8);
  unsigned int rawY = analogRead(A7);
  unsigned int  rawZ = analogRead(A9);

  float AVG_AccelDeviation[3] = {DeviationX , DeviationY, DeviationZ}; // note make sure to run before launch so that AVG_AccelDeviation could be calc
  /* ---------- methods for AVG_Accel input:------

    1. use const. obtained from previous calcs for now
    2. set to {AvgRawAccelx ,AvgRawAccely,AvgRawAccelz} or set to set to {DeviationX,DeviationY,DeviationZ}
    note make sure to run before launch so that avgRawAccel could be calc
  */
  float AccelAdjustment[3] ;
  unsigned int rawAccelArray[3] = {rawX, rawY, rawZ};
  int ii;





  if (rawX < Min_rawX)
  {
    Min_rawX = rawX;
  }
  if (rawX > Max_rawX)
  {
    Max_rawX = rawX;
  }
  if (rawY < Min_rawY)
  {
    Min_rawY = rawY;
  }
  if (rawY > Max_rawY)
  {
    Max_rawY = rawY;
  }
  if (rawZ < Min_rawZ)
  {
    Min_rawZ = rawZ;
  }
  if (rawZ > Max_rawZ)
  {
    Max_rawZ = rawZ;
  }




  AvgRawAccelx = sumX / count;

  AvgRawAccely = sumY / count;


  AvgRawAccelz = sumZ / count;


  for (ii = 0; ii < sizeof(AVG_AccelDeviation) / 4; ii++) // sizeof function shows size in #of bytes
  {




    AccelAdjustment[ii] = rawAccelArray[ii] - AVG_AccelDeviation[ii]; //-(AVG_Accel[ii]-AccelRaw_zerog);// adjustment set up so that if avg accel is >1 than  subtract and viceversa

  }




  adjusted_gAccel_X =   map_float(AccelAdjustment[0], 0, AccelRaw_MAX, -scale, scale);
  adjusted_gAccel_Y =   map_float(AccelAdjustment[1], 0, AccelRaw_MAX, -scale, scale);
  adjusted_gAccel_Z =   map_float(AccelAdjustment[2], 0, AccelRaw_MAX, -scale, scale);//template==> map_float(value,minOldRange,maxOldRange,minNewRange,MaxNewRange)
  //Serial.println("");
  if (debug == 'Y')
  {
    Serial.println(F("\n****************** ADXL377: ********************"));

    Serial.print(F("Raw Accel at 0g is:"));
    Serial.println(AccelRaw_zerog);

    // Get raw accelerometer data for each axis
    Serial.print(F("current rawAccel values are:"));
    Serial.print(rawX);
    Serial.print(", ");
    Serial.print(rawY);
    Serial.print(", ");
    Serial.println(rawZ);





    Serial.print(F("min and max rawX,Y,Z value respectively are:" ));
    Serial.print(Min_rawX);
    Serial.print("-" );
    Serial.print(Max_rawX);
    Serial.print(",  ");
    Serial.print(Min_rawY);
    Serial.print("-" );
    Serial.print(Max_rawY);
    Serial.print(",  ");
    Serial.print(Min_rawZ);
    Serial.print("-" );
    Serial.println(Max_rawZ);

    /* get AvgRawAccel values and AvgAccel values then subtract current values by that to get zero(basically make the
      AvgAccel the baseline 0; pretty sketchy calibration but its the best we got rn.

    */

    // rawAccel: currently using avg = 346,345,345 (x,y,z respectively)
    Serial.print(F("Average raw Accel(X,Y,Z) respectively are:" ));
    Serial.print(AvgRawAccelx);

    Serial.print(", " );
    Serial.print(AvgRawAccely);

    Serial.print(", " );
    Serial.println(AvgRawAccelz);

    Serial.print("The size of the avg accel array (in bytes) is:");

    Serial.println(sizeof(AVG_AccelDeviation));

    // Accel(using map function):


    Serial.print(F("Deviation X,Y,Z values respectively are:" ));
    Serial.print(DeviationX);
    Serial.print(",  " );
    Serial.print(DeviationY);
    Serial.print(", " );
    Serial.println(DeviationZ);
    Serial.print(F("The accel X.Y,Z raw adjustments are:"));
    for (ii = 0; ii < sizeof(AccelAdjustment) / 4; ii++)
    {
      Serial.print(AccelAdjustment[ii]);
      Serial.print(",  ");
    }

    Serial.print(F("\n adjusted X acceleration(g):  "));
    Serial.print(adjusted_gAccel_X);
   

    Serial.println(F(""));
    Serial.print(F("adjusted Y acceleration(g):  "));
    Serial.print(adjusted_gAccel_Y);
    
    Serial.println(F(""));


    Serial.print(F("adjusted Z acceleration(g):  "));
    Serial.print(adjusted_gAccel_Z);
    
    Serial.println(" ");



  }

  //================ apply lowpass filter: ====================================

  delay(500);


}


// FOR MPU-AXL377: arduino map function
//Converts rawAccel to volts or rawAccel to scaledAccel:
//from:https://ez.analog.com/mems/f/q-a/89030/how-to-calibrate-accelerometer-adxl377?ReplySortBy=CreatedDate&ReplySortOrder=Ascending
float map_float(float x, float minInput, float maxInput, float minOut, float maxOut)
{
  float conv;
  conv = (x - minInput) * (maxOut - minOut) / (maxInput - minInput) + minOut;
  return conv;
}

// Read "sampleSize" samples and report the average
float AVERAGE(float axisPin, int sampleSize)
{
  float reading = 0;

  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += axisPin;
  }
  float AVG_Reading = reading / sampleSize;
  return AVG_Reading;

}
// Filters Data through a low pass filter. Use "alpha" to adjust filter strength.
float LowPassFilter(float OldVal, float NewRawVal, float alpha)
{
  float ProcessedVal;
  ProcessedVal = alpha * OldVal + (NewRawVal) * (1 - alpha);
  return ProcessedVal;
}
