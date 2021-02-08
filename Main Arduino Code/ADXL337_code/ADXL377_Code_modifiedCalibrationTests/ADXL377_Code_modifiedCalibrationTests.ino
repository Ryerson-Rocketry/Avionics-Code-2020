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
// Make sure these two variables are correct for your setup
int scale = 200; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377
float AccelPin_MaxVoltage = 5.0; // max voltage for the sensor pin
float AccelRaw_MAX,AccelRaw_zerog; 
float AVGrawFluct_X,AVGrawFluct_Y,AVGrawFluct_Z;
float rawFluct_X[100],rawFluct_Y[100],rawFluct_Z[100];   float lowpass_x,lowpass_y,lowpass_z;

unsigned int sumX,sumY,sumZ=0;// unsigned int, otherwise last 7 digits makes sum <0 , also sum should never be <0 w/ raw values


int i,Maxcount=200;
float AvgRawAccelx,AvgRawAccely,AvgRawAccelz;
float deviationX,Deviation,DeviationX,SumDeviationX;
float deviationY,deviationZ,DeviationY,DeviationZ,SumDeviationY,SumDeviationZ;
float Min_rawY=512,Max_rawY=512,Min_rawZ=512,Max_rawZ=512,Min_rawX=512,Max_rawX=512;
float Min_rawY2=512,Max_rawY2=512,Min_rawZ2=512,Max_rawZ2=512,Min_rawX2=512,Max_rawX2=512;

 //METHOD: using statistics equations:
 float scaledY,scaledZ,scaledX,Xscaled;
    float accelXcount=0,Xaccel=0;
    int count,counter2=0;
unsigned int RawArray_X[200],RawArray_Y[200], RawArray_Z[200];  float AccelAdjustment[3],AccelAdjustSum[3];

//--------------------------------------------------------

void setup() {
  /*
  "analogReference(EXTERNAL)":-------------?
  
    Tells the arduino that the reference volt value is not 3.3V or 5V, when using an 
    external batt source thats not 3.3 or 5v.
   */
   //analogReference(EXTERNAL); // use if power source isnt 3.3 or 5v--------?
  Serial.begin(115200);


// -------------Assigning max volt for sensor pin:-------------
 if (AccelPin_MaxVoltage == 3.3)
{
  AccelRaw_MAX = 675; // analog reading max= (3.3/5)*(1024 bits) ,since at 5v analog pin=1023
  AccelRaw_zerog =   map_float(0,-scale,scale,0,AccelRaw_MAX);//AccelRaw_MAX/2; 
  
}
else if (AccelPin_MaxVoltage == 5.0)
{
  AccelRaw_MAX = 1024;
  AccelRaw_zerog = 512.0;//map_float(Accel_g,-scale,scale,0,AccelRaw_MAX);//AccelRaw_MAX/2; 

}
//--------------Calibrating: --------------

while(count<=Maxcount)
{
unsigned int rawX = analogRead(A0);
unsigned int rawY = analogRead(A1);
unsigned int  rawZ = analogRead(A2);   
  sumX += rawX;
  sumY += rawY;
  sumZ += rawZ;

RawArray_X[count] = sumX;
RawArray_Y[count] = sumY;
RawArray_Z[count] = sumZ;
  Serial.print(F("At count: ")); Serial.print(count);
  Serial.print(F("  The raw accel sums(X,Y,Z) are:"));
  Serial.print(RawArray_X[count]);
  Serial.print(",  ");
  Serial.print(RawArray_Y[count]);
  Serial.print(",  ");
  Serial.println(RawArray_Z[count]);  
rawFluct_X[count] = rawX; 
rawFluct_Y[count] = rawY;
rawFluct_Z[count] = rawZ; 


  /*
   * The last couple of digits printed reveals that there is something wrong
   * with serial or the code... therefore, i will add a condition below, to break the loop when 
   * the count and sum is at a specific value onwards.
  
   */

  // The condition, determining which sum gets screwed up 1st(X,Y,or Z):

if ((RawArray_X[count]<RawArray_X[count-1]))
{
   Serial.print(F("Last rawZ accel sums is invalid, at count:"));
Serial.println(count);
    Serial.print(F("The new sumX is:\t"));
  sumX=RawArray_X[count-1];
    Serial.println(sumX);

  Serial.print(F("The new sumY is:\t"));
  sumY=RawArray_Y[count-1];
    Serial.println(sumY);
Serial.print(F("The new sumZ is:\t"));
  sumZ=RawArray_Z[count-1];
    Serial.println(sumZ);
 
    Serial.print(F("rawFluct_X is:\t"));
    Serial.println(rawFluct_X[count]);
   //----determining raw fluctuations from each other-----:
    for (int ii=0;ii<=count;ii++)
    {
      AVGrawFluct_X = (rawFluct_X[ii+1]-rawFluct_X[ii]); 
Serial.print(F("The sum of fluct x is:\t")); Serial.println(AVGrawFluct_X); 
    }

// clear RawArrays to save memory:
memset(RawArray_X,0,sizeof RawArray_X);
memset(RawArray_Y,0,sizeof RawArray_Y);
memset(RawArray_Z,0,sizeof RawArray_Z);

  break;
  
  
}
if (((RawArray_Y[count]<RawArray_Y[count-1])))
{
    Serial.print(F("Last rawZ accel sums is invalid, at count:"));
Serial.println(count);
    Serial.print(F("The new sumX is: \t"));
  sumX=RawArray_X[count-1];
    Serial.println(sumX);

  Serial.print(F("The new sumY is: \t"));
  sumY=RawArray_Y[count-1];
    Serial.println(sumY);
Serial.println(F("The new sumZ is:\t"));
  sumZ=RawArray_Z[count-1];
    Serial.println(sumZ);
          //----determining raw fluctuations from each other-----:
    for (int ii=0;ii<=count;ii++)
    {
      AVGrawFluct_X = (rawFluct_X[ii+1]-rawFluct_X[ii]); 
Serial.print(F("The sum of fluct x is:\t")); Serial.println(AVGrawFluct_X); 
    }
    
// clear RawArrays to save memory:
memset(RawArray_X,0,sizeof RawArray_X);
memset(RawArray_Y,0,sizeof RawArray_Y);
memset(RawArray_Z,0,sizeof RawArray_Z);
  break;
}

if (((RawArray_Z[count]<RawArray_Z[count-1])))
{
   Serial.print(F("Last rawZ accel sums is invalid, at count:"));
Serial.println(count);
    Serial.print(F("The new sumX is: \t"));
  sumX=RawArray_X[count-1];
    Serial.println(sumX);

  Serial.print(F("The new sumY is: \t"));
  sumY=RawArray_Y[count-1];
    Serial.println(sumY);
Serial.print(F("The new sumZ is: \t"));
  sumZ=RawArray_Z[count-1];
    Serial.println(sumZ);
  //----determining raw fluctuations from each other-----:
    for (int ii=0;ii<count;ii++)
    {
      AVGrawFluct_X = abs(rawFluct_X[ii+1]-rawFluct_X[ii]); 
      float AVGrawFluct_X2;
      AVGrawFluct_X2+= AVGrawFluct_X;   
      if (ii==(count-1))
      {
      AVGrawFluct_X = AVGrawFluct_X2/(count-1);
      Serial.print(F("The sum of fluct x,y,z are:\t")); Serial.println(AVGrawFluct_X); 
      }

    }

// clear RawArrays to save memory:
memset(RawArray_X,0,sizeof RawArray_X);
memset(RawArray_Y,0,sizeof RawArray_Y);
memset(RawArray_Z,0,sizeof RawArray_Z);
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
  deviationX += abs(rawX-mew1);
  deviationY += abs(rawY-mew1);
  deviationZ += abs(rawZ-mew1);
  if (rawX-mew1 <0)
  {
    Serial.println(F("dev X <0"));
  }
  
  if (rawY-mew1 <0)
  {
    Serial.println(F("dev Y <0"));
  }
  
  if (rawZ-mew1 <0)
  {
    Serial.println(F("dev Z <0"));
  }
//Serial.println(deviationX);

// AVG. DEVIATION:
DeviationX =deviationX/count;//for accelX
DeviationY =deviationY/count;
DeviationZ =deviationZ/count;
count=count+1;
delay(100);
}

 

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
  
    Serial.print(F("Raw Accel at 0g is:"));
  Serial.println(AccelRaw_zerog);
  
 float AVG_AccelDeviation[3] = {DeviationX ,DeviationY,DeviationZ};
 /* ---------- methods for AVG_Accel input:------ 
  *  
 1. use const. obtained from previous calcs for now
 2. set to {AvgRawAccelx ,AvgRawAccely,AvgRawAccelz} or set to set to {DeviationX,DeviationY,DeviationZ}
 note make sure to run before launch so that avgRawAccel could be calc
  */


// Accel(using map function):

  
  Serial.print(F("Average Deviation X,Y,Z values respectively are:" ));  
  Serial.print(DeviationX);
      Serial.print(",  " );  
  Serial.print(DeviationY);
        Serial.print(", " );  
  Serial.println(DeviationZ);

// raw Accel is an unsigned int to make sure values received are from 0-AccelRaw_MAX 
  unsigned int rawX = analogRead(A0);
unsigned int rawY = analogRead(A1);
unsigned int  rawZ = analogRead(A2);
  
  unsigned int rawAccelArray[3] = {rawX,rawY,rawZ};
  int ii; int jj=0;
  // sizeof function shows size in #of bytes
  
  while(jj<=100)
  {
for(ii=0;ii<sizeof(AVG_AccelDeviation)/4; ii++) 
     {
   
 

    
      AccelAdjustment[ii] = rawAccelArray[ii]+AVG_AccelDeviation[ii]; //-(AVG_Accel[ii]-AccelRaw_zerog);// adjustment set up so that if avg accel is >1 than  subtract and viceversa 
      AccelAdjustment[2] = rawAccelArray[ii]-AVG_AccelDeviation[ii];
      AccelAdjustSum[ii] += AccelAdjustment[ii];
     
    
    }
    
    
  
  jj=jj+1;
  }
     
      Serial.print(F("The accel adjustment and its sum are:"));
  for (ii=0; ii<sizeof(AccelAdjustment)/4;ii++)
  {
    AccelAdjustment[ii] = AccelAdjustSum[ii]/100;
   Serial.print(AccelAdjustment[ii]);
  Serial.print(",    ");
  Serial.print(AccelAdjustSum[ii]);
  Serial.print(",    ");
  }

}


void loop() {
 // raw Accel is an unsigned int to make sure values received are from 0-AccelRaw_MAX 
  unsigned int rawX = analogRead(A0);
unsigned int rawY = analogRead(A1);
unsigned int  rawZ = analogRead(A2);
 Serial.println("");

Serial.print(F("Millis() is:"));
Serial.println(millis());
 

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

   // Get raw accelerometer data for each axis
 Serial.println("");
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
  



 

//apply my Moving average filter method:
float constantX,constantY,constantZ;
if ((rawX-AccelRaw_zerog) ==0)
{
  constantX =1; 
}
if ((rawY-AccelRaw_zerog) ==0)
{
  constantY =1; 
}
if ((rawZ-AccelRaw_zerog) ==0)
{
  constantZ =1; 
}
constantX = (1+((AccelAdjustment[0]-AccelRaw_zerog)/AccelRaw_zerog));
constantY = (1+((AccelAdjustment[1]-AccelRaw_zerog)/AccelRaw_zerog));
constantZ = (1+((AccelAdjustment[2]-AccelRaw_zerog)/AccelRaw_zerog));

 scaledX =  map_float(rawX*constantX, 0, AccelRaw_MAX, -scale, scale);
 scaledY =    map_float(rawY*constantY, 0, AccelRaw_MAX, -scale, scale);
 scaledZ =    map_float(rawZ*constantZ, 0, AccelRaw_MAX, -scale, scale);//template==> map_float(value,minOldRange,maxOldRange,minNewRange,MaxNewRange)

  
 Serial.println("");
 Serial.print(F("scaledX acceleration(g):  "));
    Serial.println(scaledX);
Serial.print(F("scaledY acceleration(g):  "));
    Serial.println(scaledY);
    Serial.print(F("scaledZ acceleration(g):  "));
    Serial.println(scaledZ);
   
    //---------Determining max and min adjusted raw accel. values:---------
  if (AccelAdjustment[0] < Min_rawX2)
  {
    Min_rawX2 = AccelAdjustment[0];
  }
  if (AccelAdjustment[0] > Max_rawX2)
  {
    Max_rawX2 = AccelAdjustment[0];
  }
  if (AccelAdjustment[1] < Min_rawY2)
  {
    Min_rawY2 = AccelAdjustment[1];
  }
  if (AccelAdjustment[1] > Max_rawY2)
  {
    Max_rawY2= AccelAdjustment[1];
  }
  if (AccelAdjustment[2] < Min_rawZ2)
  {
    Min_rawZ2 = AccelAdjustment[2];
  }
  if (AccelAdjustment[2] > Max_rawZ2)
  {
    Max_rawZ2 = AccelAdjustment[2];
  }
  
 Serial.print(F("min and max raw adjusted X,Y,Z value respectively are:") );  
  Serial.print(Min_rawX2);
      Serial.print("-" );  
  Serial.print(Max_rawX2);
  Serial.print("\t ");
  Serial.print(Min_rawY2);
      Serial.print("-" );  
  Serial.print(Max_rawY2);
  Serial.print("\t ");
  Serial.print(Min_rawZ2);
      Serial.print("-" );  
  Serial.println(Max_rawZ2);

  //apply lowpass filter:
  lowpass_x = LowPassFilter(lowpass_x, rawX, 0.5);
  
  lowpass_y = LowPassFilter(lowpass_y, rawY, 0.5);
  lowpass_z = LowPassFilter(lowpass_z, rawZ, 0.5);
Serial.print(F("the low pass raw filtered values(x,y,z) are: "));
Serial.print(lowpass_x);
Serial.print("  ");
Serial.print(lowpass_y);
Serial.print("  ");
Serial.println(lowpass_z);
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
float AVERAGE(float axisPin,int sampleSize)
{
  float reading = 0;
  
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += axisPin;
  }
 float AVG_Reading = reading/sampleSize; 
  return AVG_Reading;
 
}
// Filters Data through a low pass filter. Use "alpha" to adjust filter strength.
float LowPassFilter(float OldVal, float NewRawVal, float alpha)
{
  float ProcessedVal;
  ProcessedVal = alpha * OldVal + (NewRawVal) * (1 - alpha);
  return ProcessedVal;
}
