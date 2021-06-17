 /*-------------------------REFERENCES:---------------------------
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


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
#include <stdio.h>
int scale = 200; //this value is 200 for a 200+-g accelerometer
int samplesize = 10; //how many seconds of data collection are desired
float sampletime;
int numReadings =100;

//determining Analogue Mean==> METHOD 2: 
//from: https://forum.arduino.cc/index.php?topic=90663.0

const unsigned int numValues = 7;
uint16_t values[numValues];
uint8_t valIndex;
uint16_t sum;       // invariant: initialized => sum == + over values
bool initialized = false;
uint16_t filterInput(short sensorValue)
{
  if (initialized)
  {
    sum -= values[valIndex];
    values[valIndex] = sensorValue;
    sum += sensorValue;
    ++valIndex;
    if (valIndex = numValues)
    {
      valIndex = 0;
    }
  }
  else
  {
    for (uint8_t i = 0; i < numValues; ++i)
    {
      values[i] = sensorValue;
    }
    valIndex = 0;
    sum = sensorValue * numReadings;
    initialized = true;
  }
  return sum/numValues;
}

unsigned long sumX=0;
byte analogPinX = A0;
int i,MaxDataPts=100;
float AvgRawAccelx,DataSet_rawX[100],Min_rawX=512,Max_rawX=512;//DataSet_rawX is an array to hold 1st 100 rawX values
float deviation,deviation2,Deviation,Deviation2,Deviation3,Deviation4,SumDeviation3,SumDeviation4;
float deviationY,deviationZ,DeviationY,DeviationZ,SumDeviationY,SumDeviationZ;
float Min_rawY=512,Max_rawY=512,Min_rawZ=512,Max_rawZ=512;
// float scaledX = 0, scaledY = 0, scaledZ = 0; //TODO (JB):

void setup() {
  /*
  "analogReference(EXTERNAL)":
  
    Tells the arduino that the reference volt value is not 3.3V,but 
    rather the actual voltage given from the arduino to the components, 
    as the computer doesnt give out constant current.
   */
   //analogReference(INTERNAL); // use if power source is computer
  Serial.begin(115200);
  // bitClear(ADCSRA,ADPS0); 
  //bitSet(ADCSRA,ADPS1); 
 // bitClear(ADCSRA,ADPS2);
 double readings[100];
   //"smoothing" Analogue Mean==> initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }


    
}

void loop() {
  
  // put your main code here, to run repeatedly:
  sampletime = millis();
  float rawX = analogRead(A0);//analogRead(analogPinX);
  float rawY = analogRead(A1);
  float  rawZ = analogRead(A2);
 
   // printing multiple elements in a string/serial monitor:
  int i, string;
  char STRING[3];
  string = sprintf(STRING,"%f,%f,%f", rawX,rawY,rawZ);
  for (i=0; i <=string; i++)
  {
    Serial.println(STRING[i]);
  }
  
  Serial.println("                                  "); 
  Serial.print("The rawX is:"); Serial.print( rawX);
  Serial.print("The rawY is: "); Serial.print( rawY);
  Serial.print("The rawZ is:"); Serial.println(rawZ);
  
 Serial.print("the sampling time (in sec) is:");
  Serial.println(floor(sampletime*.001));// the 2nd argument is for sig. figs. 
  //int n=floor(sampletime*.001)/2;// setting up a counter for averaging scaled values
 Serial.print("serial iteration number is:");
  Serial.println(ceil((sampletime*.001)/2));// the 2nd argument is for sig. figs. 
//---------------------------------------------------------------------------------
// METHOD 7(hand calcs): method #1:
    float Yscaled7,Zscaled7,Xscaled8,Xscaled7,Xscaled6,Xscaled5,Xscaled4,Xscaled3,Xscaled2,Xscaled;
    float accelXcount,Xaccel=0,count=0;
   deviation=0;deviation2=0;deviationY=0;deviationZ=0; sumX=0;// i need this for some reason it reduces error by alot---?

  

 
    
  float scaledX, scaledY, scaledZ;
  scaledX = map_float(rawX, 273, 1023, -scale, scale); //1023 for 3.3V, 675(=(3.3/5)*1023) for 5V
  scaledY = map_float(rawY, 273 ,1023, -scale, scale);
  scaledZ = map_float(rawZ, 273, 1023, -scale, scale);
  
  //TODO (JB): Compare check whether new low pass filter function is a more effective way to scale acceleration values
  scaledX = LowPassFilter(scaledX, rawX, 0.9);
   scaledY = LowPassFilter(scaledY, rawY, 0.9);
   scaledZ = LowPassFilter(scaledZ, rawZ, 0.9);
  
  
 //1296 is 1023 + 273 rawX,Y,Z values found (so just adjust the max/min possible ranges to calibrate) so 0g (512 raw) is now 273 raw values
Serial.print("The scale accel (X,Y,Z) values (from adjusting raw range from 0g being 512 to 273) is:");
Serial.print(scaledX);
Serial.print(scaledY);
Serial.println(scaledZ);
 
  float zero_g_X; 
float accelX_adjusted ;
  //This part of the program scales the raw X, Y and Z values into voltage output
   
//-------------Voltage values below found using map_float function: so most likely slightly incorrect/off----------------
   float Vx, Vy, Vz;
   Vx = map_float(rawX, 0, 1024, 0, 3.3);// from: https://ez.analog.com/mems/f/q-a/89030/how-to-calibrate-accelerometer-adxl377?ReplySortBy=CreatedDate&ReplySortOrder=Ascending
   Vy = map_float(rawY, 0, 1024, 0, 3.3);
   Vz = map_float(rawZ, 0, 1024, 0, 3.3);
   // Referring to the datasheet, the no acceleration voltage is typically 1.5V and ranges from 1.4 to 1.6V.
      
    float sensitivityFactor = 0.0065;// in V/g
float accelXmoving = 560, deviation_accelX,deviation_VoltX,Avg_VoltX=1.5345;

    zero_g_X= AvgRawAccelx;

accelX_adjusted= (((rawX)-zero_g_X)*3.3/1024)/sensitivityFactor;  //from:https://forum.arduino.cc/index.php?topic=520719.0
float accelX_adjust2= scaledX-accelX_adjusted;

 Serial.print("Note: that the following voltage values are found by using the map_float function==>");
  Serial.print("Vx is: ");
  Serial.print(Vx,3); //The number after the variable (in this case: 6), decides the number of significant digits
  Serial.print(" V");
    Serial.print(", ");
  Serial.print("Vy is:");
  Serial.print(Vy,3);
  Serial.print(" V");
  Serial.print(", ");
  Serial.print("Vz is:");
  Serial.print(Vz,3);
    Serial.println(" V");

  //delay(1);
// since AXDL377 is ratiometric the goal is for Voltage = Vs/2=1.65V for each pin, giving a rawAccel value ~=512

  

  Serial.print("The adjusted accelX(g) using online method:");
  Serial.println(accelX_adjusted);
delay(2000); //min 2 milliseconds of delay 
 
}

//Converts rawAccel to volts or rawAccel to scaledAccel:
//from:https://ez.analog.com/mems/f/q-a/89030/how-to-calibrate-accelerometer-adxl377?ReplySortBy=CreatedDate&ReplySortOrder=Ascending

float map_float(float x, float minInput, float maxInput, float minOut, float maxOut)
{
  float conv;
  conv = (x - minInput) * (maxOut - minOut) / (maxInput - minInput) + minOut;
  return conv;
}



// Filters Data through a low pass filter. Use "alpha" to adjust filter strength.
float LowPassFilter(double OldVal, double NewRawVal, double alpha)
{
  float ProcessedVal;
  ProcessedVal = alpha * OldVal + (NewRawVal - OldVal) * (alpha - 1);
  return ProcessedVal;
}
