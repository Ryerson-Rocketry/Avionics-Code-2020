#include <Adafruit_GPS.h>

// NOTE: '-----?' = check/look further into

#include <Adafruit_BMP280.h>

#include "encoder_rrc_v2_4.h"

#include <stdio.h>
#include <stdint.h>

#include <Wire.h>
#include <SPI.h>

//-----------------bmp280----------------------------
/*
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
*/

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
//----------------------------------------------------

//---------------MPU-AXL377------------------------------
// Make sure these two variables are correct for your setup
int scale = 200; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377
boolean micro_is_5V = false; // Set to true if using a 5V microcontroller such as the Arduino Uno, false if using a 3.3V microcontroller, this affects the interpretation of the sensor data
//--------------------------------------------------------
unsigned long sumX=0;
byte analogPinX = A0;
int i,MaxDataPts=100;
float AvgRawAccelx,DataSet_rawX[100],Min_rawX=512,Max_rawX=512;//DataSet_rawX is an array to hold 1st 100 rawX values
float deviation,deviation2,Deviation,Deviation2,Deviation3,Deviation4,SumDeviation3,SumDeviation4;
float deviationY,deviationZ,DeviationY,DeviationZ,SumDeviationY,SumDeviationZ;
float Min_rawY=512,Max_rawY=512,Min_rawZ=512,Max_rawZ=512;


// put your setup code below, to run once:

void setup() {
  
   /*
  "analogReference(EXTERNAL)":
  
    Tells the arduino that the reference volt value is not 3.3V,but 
    rather the actual voltage given from the arduino to the components, 
    as the computer doesnt give out constant current.

   */
   analogReference(EXTERNAL); // use if power source is computer
  Serial.begin(115200); // bmp280=9600 baud rate, MPU-AXL377 = 115200 baud rate
 //pinMode(A0,INPUT); //for potentiometer
 
/*
pinMode(A4,INPUT);// see if this is needed ------------------?----conclusion==> no it's not needed
pinMode(A5,INPUT);
pinMode(A0,INPUT);
pinMode(A1,INPUT);
pinMode(A2,INPUT);
*/
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while(1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

 // bmp_temp->printSensorDetails();

 // sd-card reader code:

 // gps:

 

}

 // put your main code below, to run repeatedly:

void loop() {
   sampletime = millis();

   // Get raw accelerometer data for each axis
  int rawX = analogRead(A0);
  int rawY = analogRead(A1);
  int rawZ = analogRead(A2);
  
 Serial.print("the sampling time (in sec) is:");
  Serial.println(floor(sampletime*.001));// the 2nd argument is for sig. figs. 
Serial.print("serial iteration number is:");
  Serial.println(ceil((sampletime*.001)/2));// the 2nd argument is for sig. figs. 

 //METHOD: using statistics equations:
 float Yscaled7,Zscaled7,Xscaled8,Xscaled7,Xscaled6,Xscaled5,Xscaled4,Xscaled3,Xscaled2,Xscaled;
    float accelXcount,Xaccel=0,count=0;
   deviation=0;deviation2=0;deviationY=0;deviationZ=0; sumX=0;// i need this for some reason it reduces error by alot---?
while(count<=100)
{
  sumX += analogRead(analogPinX);
  
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
  float mew1 = 512;//,mew2 = sumX/(count+1);
  deviation += pow(abs(rawX-mew1),2);
  deviationY += pow(abs(rawY-mew1),2);
  deviationZ += pow(abs(rawZ-mew1),2);
   

count=count+1;
delay(1);
}
 Serial.print("min and max rawX,Y,Z value respectively are:" );  
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
  
AvgRawAccelx = sumX / 100;
Deviation3 =sqrt(deviation/100);//for accelX
DeviationY =sqrt(deviationY/100);
DeviationZ =sqrt(deviationZ/100);

SumDeviation3 +=Deviation3;
SumDeviationY +=DeviationY;
SumDeviationZ +=DeviationZ;

  Serial.print("Deviation X,Y,Z values respectively are:" );  
  Serial.print(Deviation3);
      Serial.print(",  " );  
  Serial.print(DeviationY);
        Serial.print(", " );  
  Serial.print(DeviationZ);

   Serial.print("And their sums are:" );  
  Serial.print(SumDeviation3);
      Serial.print(", " );  
  Serial.print(SumDeviationY);
  
      Serial.print(", " );  
  Serial.println(SumDeviationZ);
  
 Xscaled7 =   map_float(rawX-Deviation3, 0, 1024, -scale, scale);
       Yscaled7 =   map_float(rawY-DeviationY, 0, 1024, -scale, scale);
       Zscaled7 =   map_float(rawZ-DeviationZ, 0, 1024, -scale, scale);

      float revXscaledraw,revYscaledraw,revZscaledraw;
      revXscaledraw = map_float(Xscaled7,-200,200,0,1024);
      revYscaledraw = map_float(Yscaled7,-200,200,0,1024);
      revZscaledraw = map_float(Zscaled7,-200,200,0,1024);
     
      Serial.print("reversed scaled accel X,Y,Z to raw accel X,Y,Z are:");
      Serial.print(revXscaledraw); 
      Serial.print(",   ");
      Serial.print(revYscaledraw); 
      Serial.print(",   ");
       Serial.print(revZscaledraw); 
           
  float correctionFactor_X=0.4,correctionFactor_Y=0.3,correctionFactor_Z=0.35;
  float Corrected_Xscaled7,Corrected_Yscaled7,Corrected_Zscaled7;
  Corrected_Xscaled7 =   map_float(rawX-Deviation3+correctionFactor_X, 0, 1024, -scale, scale);
  Corrected_Yscaled7 =   map_float(rawY-DeviationY+correctionFactor_Y, 0, 1024, -scale, scale);
Corrected_Zscaled7 =   map_float(rawZ-DeviationZ+correctionFactor_Z, 0, 1024, -scale, scale);

      Serial.print("method A-2:(subtracting): corrected accel X,Y,Z values are:" );  
  Serial.print(Corrected_Xscaled7);
Serial.print(",   ");  
  Serial.print(Corrected_Yscaled7);
Serial.print(",   ");
    Serial.println(Corrected_Zscaled7);
   
// --------------applying the encoder:-------------------
  //float temp;//,poten;
  int i,t=0;
  uint32_t int_data;
  uint8_t encpoten[8],encTemp[8],encPress[8],encAlt[8],encAccelX[8],encAccelY[8],encAccelZ[8];
  // check why can you initialize an array like done above (w/o malloc or calloc)-------?
  
  /*
   //potentiometer code:
  poten = analogRead(A0);
  Serial.println();

  Serial.print("poten");
    Serial.println();

    Serial.print(poten);
  Serial.println(" = ");
  encode(poten, header, t,encpoten);
 for (i=0;i<8;i++)
  {
    Serial.print(encpoten[i],HEX);
   }
*/

//adafruit gps:


//bmp280 temperature:
  Serial.println();
      Serial.print(F("bmp temp(C):  "));
      //temp = bmp.readTemperature();
   // Serial.print(temp);
        Serial.print(bmp.readTemperature());
  Serial.print(" = ");
  encode(bmp.readTemperature(), 0x06, t, encTemp);
   for (i=0;i<8;i++)
  {
    Serial.print(encTemp[i],HEX);
  }
  Serial.println();

  // bmp pressure;
      Serial.print(F("bmp pressure(Pa):  "));
    Serial.print(bmp.readPressure());
  Serial.print(" = ");
  encode(bmp.readPressure(),0x07, t, encPress);
   for (i=0;i<8;i++)
  {
    Serial.print(encPress[i],HEX);
  }
  Serial.println();

//bmp altitude:
      Serial.print(F("bmp alt(m):  "));
    Serial.print(bmp.readAltitude());
  Serial.print(" = ");
  encode(bmp.readAltitude(),0x00 , t, encAlt);
  //Serial.println();


// printing the encoder values, bit by bit: 
  for (i=0;i<8;i++)
  {
    Serial.print(encAlt[i],HEX);
  
  }
// MPU-AXL377:
   
    Serial.print(F("scaledX acceleration(g):  "));
    Serial.print(Corrected_Xscaled7);
  Serial.print(" = ");
  encode(Corrected_Xscaled7,0x03, t, encAccelX);
   for (i=0;i<8;i++)
  {
    Serial.print(encAccelX[i],HEX);
  }
 
        Serial.println(F(""));
    Serial.print(F("scaledY acceleration(g):  "));
    Serial.print(Corrected_Yscaled7);
  Serial.print(" = ");
  encode(Corrected_Yscaled7,0x04, t, encAccelY);
   for (i=0;i<8;i++)
  {
    Serial.print(encAccelY[i],HEX);
  }
      Serial.println(F(""));

 
    Serial.print(F("scaledZ acceleration(g):  "));
    Serial.print(Corrected_Zscaled7);
  Serial.print(" = ");
  encode(Corrected_Zscaled7,0x05, t, encAccelZ);
   for (i=0;i<8;i++)
  {
    Serial.print(encAccelZ[i],HEX);
  }
  /* a for loop needs to be called right after calling the encode function to print encoded values
  bit by bit, as the encoded values obtained are not saved -------==> look at encAlt,encPress,etc. -------------?
  */ 
        t++;
delay(2000);// for MPU-AXL377: Minimum delay of 2 milliseconds between sensor reads (500 Hz)
}

// FOR MPU-AXL377: Same functionality as Arduino's standard map function, except using floats
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//Converts rawAccel to volts or rawAccel to scaledAccel:
//from:https://ez.analog.com/mems/f/q-a/89030/how-to-calibrate-accelerometer-adxl377?ReplySortBy=CreatedDate&ReplySortOrder=Ascending
float map_float(float x, float minInput, float maxInput, float minOut, float maxOut)
{
  float conv;
  conv = (x - minInput) * (maxOut - minOut) / (maxInput - minInput) + minOut;
  return conv;
}

// readAltitude is ReadAltitude( ijust copied and pasted it to look at it easier)==> this makes it so that the alt header is equal to the press header(reducing #headers needed):

float ReadAltitude(float seaLevelhPa) {
  float altitude;
  float pressure = bmp.readPressure(); // in Si units for Pascal
  pressure /= 100;
  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
  return altitude;
}


// Read "sampleSize" samples and report the average
int ReadAxis(int axisPin)
{
  long reading = 0;
  int sampleSize=300;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
 
  return reading/sampleSize;
 
}
