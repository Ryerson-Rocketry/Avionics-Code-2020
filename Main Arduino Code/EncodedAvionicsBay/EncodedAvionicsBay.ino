// NOTE: '-----?' = check/look further into

// hardware serial is for MEGA (USED UNO) so set up may be slightly different 

#include <encoder_rrc_v2_4.h>


#include <Adafruit_MPL3115A2.h>
#include <Adafruit_BMP280.h>



#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <Wire.h>
#include <SPI.h>

//-----------------bmp280----------------------------

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

//SDA - A4(Uno),20(mega), SCL-A5(uno),21(mega)

/* -------------------if using SPI:
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
*/


//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
//--------------------MPL311A2 BAROMETER:--------------------------------

// Power by connecting Vin to 3-5V, GND to GND
// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
// See the Wire tutorial for pinouts for each Arduino
// http://arduino.cc/en/reference/wire
//Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
//---------------------Adafruit Ultimate GPS: ---------------
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define GPSSerial Serial1 /// Serial1 =(gpsTX to pin19, GpsRX to pin18), serial2 = rx-17,tx-16,serial3 = rx-15,tx-14

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);


// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7
//SoftwareSerial mySerial(4, 3);

//Adafruit_GPS GPS(&mySerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences (seeing GPS data sent to arduino, and the GPS data arduino received from the GPS) 
 
#define GPSECHO  false
/* --------------------for LOCUS datlogging:
 *  // this keeps track of whether we're using the interrupt
// off by default!
#ifndef ESP8266 // Sadly not on ESP8266
bool usingInterrupt = false;
#endif
 *  
 */


//---------------MPU-AXL377------------------------------
// Make sure these two variables are correct for your setup
int scale = 200; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377
float micro_voltage = 5.0; // 5 if using a 5V microcontroller such as the Arduino Uno/MEGA, 3.3 if using a 3.3V microcontroller(teensy), this affects the interpretation of the sensor data
//--------------------------------------------------------
float AccelRaw_MAX,AccelRaw_zerog; 


unsigned int sumX,sumY,sumZ=0;// unsigned int, otherwise last 7 digits makes sum <0 , also sum should never be 0 w/ raw values


int i,Maxcount=400;
float AvgRawAccelx,AvgRawAccely,AvgRawAccelz;
float deviationX,Deviation,DeviationX,SumDeviationX;
float deviationY,deviationZ,DeviationY,DeviationZ,SumDeviationY,SumDeviationZ;
float Min_rawY=512,Max_rawY=512,Min_rawZ=512,Max_rawZ=512,Min_rawX=512,Max_rawX=512;


 //METHOD: using statistics equations:
 float Yscaled7,Zscaled7,Xscaled7,Xscaled;
    float accelXcount,Xaccel=0;
    int count=0;
unsigned int RawArray_X[200],RawArray_Y[200], RawArray_Z[200];
//--------------------------------------------------------


void setup() {
  
   /*
  "analogReference(EXTERNAL)":-------------?
  
    Tells the arduino that the reference volt value is not 3.3V or 5V, when using an 
    external batt source thats not 3.3 or 5v.
   */
   //analogReference(EXTERNAL); // use if power source isnt 3.3 or 5v--------?
  Serial.begin(115200); // bmp280=9600 baud rate, MPU-AXL377 = 115200 baud rate
 bmp.begin(9600);
  // bitClear(ADCSRA,ADPS0); 
  //bitSet(ADCSRA,ADPS1); 
 // bitClear(ADCSRA,ADPS2);


 // ===============-sd-card reader code:==================

  /*
   ============================= NOTES ON GPS Protocols:===================
   
  a.  On NMEA protocols:
 https://www.gpsinformation.org/dale/nmea.htm
 https://en.wikipedia.org/wiki/NMEA_0183#:~:text=The%20NMEA%200183%20standard%20uses,%22listeners%22%20at%20a%20time.&text=The%20NMEA%20standard%20is%20proprietary,NMEA)%20as%20of%20November%202017.
b.  On geometric DOP(Dilution of Precision): https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
 
 Uses a specific serial interfacing protocol to communicate with microcontrollers called NMEA0183/NMEA2000.
 NMEA protocol consists of having 5 letters before the numerical data which defines the sentence: the 1st  two letters are GP, followed by 3 letters which indicate a specific sentence.
 Common Protocol fields are:
o GPGSA: lists the #satellites used for determining a fix position and gives a geometric DOP fix(Dilution of Precision; specifies the error propagation due to satellite geometry and positional measurement precision) ; uses 12 spaces for satellite numbers.  
o GPGSV: tells one about the satellites in view that it might be able to locate based on its viewing mask and almanac data; also tells one about the signal strength (SNR;signal to noise ratio);can only provide data to up to 4 satellites. 
o GPRMC: NMEA has its own version of essential gps pvt (position, velocity, time) data. It is called RMC, The Recommended Minimum
o GPGGA: essentially fix data which provides 3D location and its accuracy. 
*/ 


 
 //==========barometric sensor:================
 


  // Default settings from datasheet. 
  
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     //Operating Mode. 
                  Adafruit_BMP280::SAMPLING_X2,     //Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling 
                  Adafruit_BMP280::FILTER_X16,      //Filtering. 
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time. 

 //bmp280: bmp_temp->printSensorDetails();
 if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
 // ==============ADXL377 accelerometer:===================
 
            // -------------Assigning microcontroller type:-------------
 if (micro_voltage = 5.0)
{
  AccelRaw_MAX = 675;
  AccelRaw_zerog = AccelRaw_MAX/2;
  
}
else if (micro_voltage = 3.3)
{
  AccelRaw_MAX = 1023;
  AccelRaw_zerog = AccelRaw_MAX/2; 

}
//--------------Calibrating: --------------
unsigned int rawX = analogRead(A0);
unsigned int rawY = analogRead(A1);
unsigned int  rawZ = analogRead(A2);
while(count<=Maxcount)
{

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
  
  /*
   * The last couple of digits printed reveals that there is something wrong
   * with serial or the code... therefore, i will add a condition below, to break the loop when 
   * the count and sum is at a specific value onwards.
  
   */

  // The condition, determining which sum gets screwed up 1st(X,Y,or Z):

if ((RawArray_X[count]<RawArray_X[count-1]))
{
   Serial.println(F("Last rawZ accel sums is invalid, at count:"));
Serial.println(count);
    Serial.println(F("The new sumX is:"));
  sumX=RawArray_X[count-1];
    Serial.print(sumX);

  Serial.println(F("The new sumY is:"));
  sumY=RawArray_Y[count-1];
    Serial.print(sumY);
Serial.println(F("The new sumZ is:"));
  sumZ=RawArray_Z[count-1];
    Serial.print(sumZ);
// clear RawArrays to save memory:
memset(RawArray_X,0,sizeof RawArray_X);
memset(RawArray_Y,0,sizeof RawArray_Y);
memset(RawArray_Z,0,sizeof RawArray_Z);

  
  break;
  
  
}
if (((RawArray_Y[count]<RawArray_Y[count-1])))
{
    Serial.println(F("Last rawZ accel sums is invalid, at count:"));
Serial.println(count);
    Serial.println(F("The new sumX is:"));
  sumX=RawArray_X[count-1];
    Serial.print(sumX);

  Serial.println(F("The new sumY is:"));
  sumY=RawArray_Y[count-1];
    Serial.print(sumY);
Serial.println(F("The new sumZ is:"));
  sumZ=RawArray_Z[count-1];
    Serial.print(sumZ);
// clear RawArrays to save memory:
memset(RawArray_X,0,sizeof RawArray_X);
memset(RawArray_Y,0,sizeof RawArray_Y);
memset(RawArray_Z,0,sizeof RawArray_Z);
  break;
}

if (((RawArray_Z[count]<RawArray_Z[count-1])))
{
   Serial.println(F("Last rawZ accel sums is invalid, at count:"));
Serial.println(count);
    Serial.println(F("The new sumX is:"));
  sumX=RawArray_X[count-1];
    Serial.print(sumX);

  Serial.println(F("The new sumY is:"));
  sumY=RawArray_Y[count-1];
    Serial.print(sumY);
Serial.println(F("The new sumZ is:"));
  sumZ=RawArray_Z[count-1];
    Serial.print(sumZ);
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
  deviationX += rawX-mew1;
  deviationY += rawY-mew1;
  deviationZ += rawZ-mew1;
//Serial.println(deviationX);

// AVG. DEVIATION:
DeviationX =deviationX/count;//for accelX
DeviationY =deviationY/count;
DeviationZ =deviationZ/count;
count=count+1;
delay(100);
}
   // Get raw accelerometer data for each axis
 
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
  
  
 //==================GPS:==================
 
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  
  GPS.begin(9600);

  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);
// Ask for firmware version
  //GPSSerial.println(PMTK_Q_RELEASE); // GPSSerial for MEGA,mySerial for uno

 
  delay(500);

}

 // put your main code below, to run repeatedly:
//uint32_t timer = millis();

void loop() {
    
    //===========================GPS:=======================
  
  // read data from the GPS in the 'main loop'
  while (GPS.available()>0)
  { 
  char c = GPS.read();
     Serial.write(c);
       Serial.flush();//wait until GPS is finished being written to serial then proceed with other data


  
  }
     if (GPS.fix) {
      Serial.println(" ");
      Serial.println(F("GPS FIX"));

    }
  Serial.println(" ");
  
if (GPS.newNMEAreceived())
{
if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
  {
  GPS.parse(GPS.lastNMEA()); 
    Serial.flush();//wait until GPS is finished being written to serial then proceed with other data
 
  //Serial.println(GPS.lastNMEA());
  }
}


  // =============Applying the encoder:===============

  int i,t=0;
  uint32_t int_data;
  uint8_t encGPS_lat[8],encGPS_long[8],encpoten[8],encTemp[8],encPress[8],encAlt[8],encAccelX[8],encAccelY[8],encAccelZ[8];
  
  

//Adafruit GPS:
 

   Serial.print(F(" GPS lat:  "));
      //temp = bmp.readTemperature();
   // Serial.print(temp);
        Serial.print(GPS.latitude);
        Serial.print(GPS.lat);
  Serial.print(F(" = "));
  encode(GPS.latitude, 0x01, t, encGPS_lat);
   for (i=0;i<8;i++)
  {
    Serial.print(encGPS_lat[i],HEX);
  }
  Serial.println(" ");
 Serial.print(F(" GPS long:  "));
   
        Serial.print(GPS.longitude);
        Serial.print(GPS.lon);
  Serial.print(F(" = "));
  encode(GPS.longitude, 0x02, t, encGPS_long);
   for (i=0;i<8;i++)
  {
    Serial.print(encGPS_long[i],HEX);
  }


   Serial.println(" ");
Serial.print(F("GPS #satillites and angle respectfully are:   "));

Serial.print(GPS.satellites);
Serial.print(",     ");
Serial.println(GPS.angle);

//=========================== ADXL377 accelerometer:========================

  unsigned int rawX = analogRead(A0);
unsigned int rawY = analogRead(A1);
unsigned int  rawZ = analogRead(A2);
 Serial.println("");

 

 Serial.print(F("Raw Accel at 0g is:"));
  Serial.println(AccelRaw_zerog);
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
  
/* get AvgRawAccel values and AvgAccel values then subtract current values by that to get zero(basically make the 
  AvgAccel the baseline 0; pretty sketchy calibration but its the best we got rn. 

*/

// rawAccel: currently using avg = 346,345,345 (x,y,z respectively)


AvgRawAccelx = sumX / count;

AvgRawAccely = sumY / count;


AvgRawAccelz = sumZ / count;

  Serial.print(F("Average raw Accel(X,Y,Z) respectively are:" ));  
  Serial.print(AvgRawAccelx);

  Serial.print(", " );  
  Serial.print(AvgRawAccely);

    Serial.print(", " );  
  Serial.println(AvgRawAccelz);
 float AVG_AccelDeviation[3] = {DeviationX ,DeviationY,DeviationZ};// note make sure to run before launch so that AVG_AccelDeviation could be calc
   /* ---------- methods for AVG_Accel input:------ 
  *  
 1. use const. obtained from previous calcs for now
 2. set to {AvgRawAccelx ,AvgRawAccely,AvgRawAccelz} or set to set to {DeviationX,DeviationY,DeviationZ}
 note make sure to run before launch so that avgRawAccel could be calc
  */
  
  Serial.print("The size of the avg accel array (in bytes) is:");

  Serial.println(sizeof(AVG_AccelDeviation));

// Accel(using map function):

  
  Serial.print(F("Deviation X,Y,Z values respectively are:" ));  
  Serial.print(DeviationX);
      Serial.print(",  " );  
  Serial.print(DeviationY);
        Serial.print(", " );  
  Serial.println(DeviationZ);
   float AccelAdjustment[3] ;
  unsigned int rawAccelArray[3] = {rawX,rawY,rawZ};
  int ii;
  for(ii=0;ii<sizeof(AVG_AccelDeviation)/4; ii++) // sizeof function shows size in #of bytes
  {
   
 

    
      AccelAdjustment[ii] = rawAccelArray[ii]-AVG_AccelDeviation[ii]; //-(AVG_Accel[ii]-AccelRaw_zerog);// adjustment set up so that if avg accel is >1 than  subtract and viceversa 
      
  }
      Serial.print(F("The accel adjustment is:"));
  for (ii=0; ii<sizeof(AccelAdjustment)/4;ii++)
  {
   Serial.print(AccelAdjustment[ii]);
  Serial.print(",   ");
  }
 
 Xscaled7 =   map_float(AccelAdjustment[0], 0, AccelRaw_MAX, -scale, scale);
 Yscaled7 =   map_float(AccelAdjustment[1], 0, AccelRaw_MAX, -scale, scale);
 Zscaled7 =   map_float(AccelAdjustment[2], 0, AccelRaw_MAX, -scale, scale);//template==> map_float(value,minOldRange,maxOldRange,minNewRange,MaxNewRange)
 Serial.println("");
   // =============Applying the encoder:===============

// ADXL377:
   
    Serial.print(F("scaledX acceleration(g):  "));
    Serial.print(Xscaled7);
  Serial.print(" = ");
  encode(Xscaled7,0x03, t, encAccelX);
   for (i=0;i<8;i++)
  {
    Serial.print(encAccelX[i],HEX);
  }
 
        Serial.println(F(""));
    Serial.print(F("scaledY acceleration(g):  "));
    Serial.print(Yscaled7);
  Serial.print(" = ");
  encode(Yscaled7,0x04, t, encAccelY);
   for (i=0;i<8;i++)
  {
    Serial.print(encAccelY[i],HEX);
  }
      Serial.println(F(""));

 
    Serial.print(F("scaledZ acceleration(g):  "));
    Serial.print(Zscaled7);
  Serial.print(F(" = "));
  encode(Zscaled7,0x05, t, encAccelZ);
   for (i=0;i<8;i++)
  {
    Serial.print(encAccelZ[i],HEX);
  }
  Serial.println(" ");

  
     // =============Applying the encoder:===============


  

  //bmp280 temperature:
  Serial.println();
      Serial.print(F("bmp temp(C):  "));
      //temp = bmp.readTemperature();
   // Serial.print(temp);
        Serial.print(bmp.readTemperature());
  Serial.print(F(" = "));
  encode(bmp.readTemperature(), 0x06, t, encTemp);
   for (i=0;i<8;i++)
  {
    Serial.print(encTemp[i],HEX);
  }
  Serial.println();

  // bmp pressure;
      Serial.print(F("bmp pressure(Pa):  "));
    Serial.print(bmp.readPressure());
  Serial.print(F(" = "));
  encode(bmp.readPressure(),0x07, t, encPress);
   for (i=0;i<8;i++)
  {
    Serial.print(encPress[i],HEX);
  }
  Serial.println();

//bmp altitude:
      Serial.print(F("bmp alt(m):  "));
    Serial.print(bmp.readAltitude());// 1013.25 is SL pressure in hPa
  Serial.print(F(" = "));
  encode(bmp.readAltitude(),0x00 , t, encAlt);
  
  

// printing the encoder values, bit by bit: 
  for (i=0;i<8;i++)
  {
    Serial.print(encAlt[i],HEX);
  
  }


  Serial.println();


// printing the encoder values, bit by bit: 
 
 
 
// MPU gyro:

  
        t++;
delay(500);// for MPU-AXL377: Minimum delay of 2 milliseconds between sensor reads (500 Hz)
//}

}
//==================================FUNCTIONS:=================================

// FOR MPU-AXL377: arduino map function 
//Converts rawAccel to volts or rawAccel to scaledAccel:
//from:https://ez.analog.com/mems/f/q-a/89030/how-to-calibrate-accelerometer-adxl377?ReplySortBy=CreatedDate&ReplySortOrder=Ascending
float map_float(float x, float minInput, float maxInput, float minOut, float maxOut)
{
  float conv;
  conv = (x - minInput) * (maxOut - minOut) / (maxInput - minInput) + minOut;
  return conv;
}

// readAltitude is ReadAltitude( ijust copied and pasted it to look at it easier)==> this makes it so that the alt header is equal to the press header(reducing #headers needed):


// Read "sampleSize" samples and report the average
float AVG_Analog(float axisPin,int sampleSize)
{
  float reading = 0;
  
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
 float AVG_Reading = reading/sampleSize; 
  return AVG_Reading;
 
}

// readAltitude is ReadAltitude( ijust copied and pasted it to look at it easier)==> this makes it so that the alt header is equal to the press header(reducing #headers needed):

float ReadAltitude(float seaLevelhPa) {
  float altitude;
  float pressure = bmp.readPressure(); // in Si units for Pascal
  pressure /= 100;
  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
  return altitude;
}
