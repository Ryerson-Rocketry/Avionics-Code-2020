// NOTE: '-----?' = check/look further into


#include <encoder_rrc_v2_4.h>


#include <Adafruit_BMP280.h>



#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <Wire.h>
#include <SPI.h>

// ***************************** DEFINING GLOBAL VARIABLES:****************************************

//==================== 1. BMP280=================================

// ******************** I2C PACKET STRUCTURE: ******************

/*
  The I2C hanshake process is as follows(the wire function calls written next to bit indices arent confirmed yet
  and is just a theory/my ideas on the matter):

   Master:

  [Start bit(Wire.begin()), Address byte(Wire.beginTransmission(Address)),
  Read/Write bit(Wire.requestFrom(addr,#bytes to receive/read)/Wire.write()), ACK/NACK bit, data byte 1(for Write:usually used as a
  register byte(think of like a specific address we want to read/write data to/from))(both read and write use Wire.write() to signify which register we want to deal w/),
  ACK/NACK bit,data byte 2(typically the actual data to write to the register)(for reading: indicates the # of bytes to receive(Wire.requestFrom()), ACK/NACK bit, Stop bit]



   **NOTE***:
        ==> ACK/NACK bits are sent by the slave
        ==> Wire.requestFrom() for reading and Wire.write() for writing adds data to their respective buffer
          ==> Whereas, Wire.read() for reading and Wire.endTransmission(false or true) releases the data from their respective buffers
            ==>Wire.endTransmission(false) maybe gives a terminator-Stop bit which sends the previous Wire.write() buffer then sends a Start bit to initialize transmission once more.

        ==> Both Wire.write() and Wire.read()deals with writing/reading 1 byte at a time




*/
//*****************************************************************

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
// ==================== 2. GPS:===========================


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

//#include <TinyGPS++.h>

#define GPS Serial2 //(serial2= TX-,RX-7)
#define PMTK_SET_NMEA_UPDATERATE_1HZ "$PMTK220,1000*1F\r\n"
#define PMTK_SET_NMEA_UPDATERATE_5HZ "$PMTK220,200*2C\r\n"
#define PMTK_SET_NMEA_UPDATERATE_10HZ "$PMTK220,100*2F\r\n"

int pos;
int stringplace = 0;
float GPS_latitude, GPS_longitude;

String nmea[15];
String labels[12] {"Time:\t", "Status:\t", "Latitude:\t", "Hemisphere:\t", "Longitude:\t", "Hemisphere:\t", "Speed:\t", "Track Angle:\t", "Date:\t"};


//uint32_t timer = millis();

//=================== 3. ADXL377=============================

// Make sure these two variables are correct for your setup
int scale = 200; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377
float micro_voltage = 5.0; // 5 if using a 5V microcontroller such as the Arduino Uno/MEGA, 3.3 if using a 3.3V microcontroller(teensy), this affects the interpretation of the sensor data
float AccelRaw_MAX, AccelRaw_zerog;


unsigned int sumX, sumY, sumZ = 0; // unsigned int, otherwise last 7 digits makes sum <0 , also sum should never be< 0 w/ raw values


int i, Maxcount = 200;
float AvgRawAccelx, AvgRawAccely, AvgRawAccelz;
float deviationX, deviationY, deviationZ, Deviation, DeviationX , DeviationY, DeviationZ, SumDeviationX, SumDeviationY, SumDeviationZ;

float Min_rawY = 512, Max_rawY = 512, Min_rawZ = 512, Max_rawZ = 512, Min_rawX = 512, Max_rawX = 512;


//METHOD: using statistics equations:
float adjusted_gAccel_X, adjusted_gAccel_Y, adjusted_gAccel_Z;
float accelXcount, Xaccel = 0;
int count = 0;
unsigned int RawArray_X[200], RawArray_Y[200], RawArray_Z[200];

//============================= Shared Global Variables(Global Variables used by all sensors): =============================================

char debug = 'Y';// 'Y' if debugging and printing all testing values to serial monitor
int t = 0;
//==========================================================================================================================================


void setup() {

  /*
    "analogReference(EXTERNAL)":-------------?

    Tells the arduino that the reference volt value is not 3.3V or 5V, when using an
    external batt source thats not 3.3 or 5v.
  */
  //analogReference(EXTERNAL); // use if power source isnt 3.3 or 5v--------?
  Serial.begin(115200); // bmp280=9600 baud rate, MPU-AXL377 = 115200 baud rate
  bmp.begin(9600);


  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  while (Serial.available() == 0)
  { //doesnt do anything until detects buffer in serial ports
    Serial.println(F("Send byte to start data processing"));
    delay(5000);
  }
  // =============== APPLY PMTK COMMANDS: ===============
  /* Serial.write is for sending BIN data, doesnt work with Serial.print but
    Serial.printf(formatted string, and in standard(stdio.h) lib & doesnt specify new line automatically) is for sending strings/chars
  */

  GPS.printf(PMTK_SET_NMEA_UPDATERATE_1HZ);
  GPS.flush();


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



  //========== BMP280:================



  // Default settings from datasheet.

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     //Operating Mode.
                  Adafruit_BMP280::SAMPLING_X2,     //Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      //Filtering.
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time.

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  // ==============ADXL377 accelerometer:===================

  // -------------Assigning microcontroller type:-------------
  if (micro_voltage = 5.0)
  {
    AccelRaw_MAX = 1024;
    AccelRaw_zerog = AccelRaw_MAX / 2;

  }
  else if (micro_voltage = 3.3)
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
    if (debug == 'Y')
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
  if (debug == 'Y')
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

// put your main code below, to run repeatedly:
//uint32_t timer = millis();

void loop() {


  //===========================GPS:=======================

  // read data from the GPS in the 'main loop'
  char GPSdebug = 'N';
  if (GPSdebug == 'Y')
  {
    Serial.println("GPS DEBUGGING INITIATED: SEEING NMEA CODES");

  }
  while (GPS.available() > 0)
  {
    char c = GPS.read();

    // debugging to see NMEA codes:
    if (GPSdebug == 'Y')
    {
      Serial.write(c);
      Serial.flush();//wait until GPS is finished being written to serial then proceed with other data

    }
  }


  if (GPS.find("$GPRMC,")) {
    String tempMsg = GPS.readStringUntil('\n');
    for (int i = 0; i < tempMsg.length(); i++) {
      if (tempMsg.substring(i, i + 1) == ",") {// if char in string=',' from i to i+1 position:
        nmea[pos] = tempMsg.substring(stringplace, i);
        stringplace = i + 1; //stringplace is used to get all characters within two "," thus used as another counter
        pos++;
      }
      if (i == tempMsg.length() - 1) {
        nmea[pos] = tempMsg.substring(stringplace, i);
      }
    }

    // ===Add to check if data = valid (so if GPRMC contains "A"):===

    //==============================================================

    GPS_latitude = nmea[2].toFloat();
    GPS_longitude = nmea[4].toFloat();



  }

  else {
    Serial.println("GPRMC NMEA CODE NOT DETECTED!");

  }
  stringplace = 0;
  pos = 0;






  //=========================== ADXL377 accelerometer:========================
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
  // =============Applying the encoder:===============


  int i;
  uint32_t int_data;
  uint8_t encGPS_lat[8], encGPS_long[8], encTemp[8], encPress[8], encAlt[8], encAccelX[8], encAccelY[8], encAccelZ[8];



  encode(GPS_latitude, 0x01, t, encGPS_lat);
  encode(GPS_longitude, 0x02, t, encGPS_long);
  encode(adjusted_gAccel_X, 0x03, t, encAccelX);
  encode(adjusted_gAccel_Y, 0x04, t, encAccelY);
  encode(adjusted_gAccel_Z, 0x05, t, encAccelZ);

  encode(bmp.readTemperature(), 0x06, t, encTemp);
  encode(bmp.readPressure(), 0x07, t, encPress);
  encode(bmp.readAltitude(), 0x00 , t, encAlt);

  //======Printing values===============================
  if (debug == 'Y')
  {
    //=========== GPS=====================
    Serial.println(F("\n****************** GPS: ********************"));
    Serial.print(labels[2]);
    Serial.println(GPS_latitude);

    Serial.print(labels[4]);
    Serial.println(GPS_longitude);


    Serial.print(F(" GPS lat:  "));

    Serial.print(GPS_latitude);
    Serial.print(F(" = "));
    for (i = 0; i < 8; i++)
    {
      Serial.print(encGPS_lat[i], HEX);
    }
    Serial.println(" ");



    Serial.print(F(" GPS long:  "));

    Serial.print(GPS_longitude);
    Serial.print(F(" = "));
    for (i = 0; i < 8; i++)
    {
      Serial.print(encGPS_long[i], HEX);
    }



    // ADXL377:
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
    Serial.print(" = ");
    for (i = 0; i < 8; i++)
    {
      Serial.print(encAccelX[i], HEX);
    }


    Serial.println(F(""));
    Serial.print(F("adjusted Y acceleration(g):  "));
    Serial.print(adjusted_gAccel_Y);
    Serial.print(" = ");
    for (i = 0; i < 8; i++)
    {
      Serial.print(encAccelY[i], HEX);
    }
    Serial.println(F(""));


    Serial.print(F("adjusted Z acceleration(g):  "));
    Serial.print(adjusted_gAccel_Z);
    Serial.print(F(" = "));
    for (i = 0; i < 8; i++)
    {
      Serial.print(encAccelZ[i], HEX);
    }
    Serial.println(" ");






    //bmp280 temperature:

    Serial.println(F("****************** BMP280: ********************"));
    Serial.print(F("bmp temp(C):  "));
    //temp = bmp.readTemperature();
    // Serial.print(temp);
    Serial.print(bmp.readTemperature());
    Serial.print(F(" = "));
    for (i = 0; i < 8; i++)
    {
      Serial.print(encTemp[i], HEX);
    }
    Serial.println();


    // bmp pressure;

    Serial.print(F("bmp pressure(Pa):  "));
    Serial.print(bmp.readPressure());
    Serial.print(F(" = "));
    for (i = 0; i < 8; i++)
    {
      Serial.print(encPress[i], HEX);
    }
    Serial.println();

    //bmp altitude:

    Serial.print(F("bmp alt(m):  "));
    Serial.print(bmp.readAltitude());// 1013.25 is SL pressure in hPa
    Serial.print(F(" = "));



    // printing the encoder values, bit by bit:
    for (i = 0; i < 8; i++)
    {
      Serial.print(encAlt[i], HEX);

    }



    Serial.print("\n t is:\t");
    Serial.println(t);
  }
  t++;



  //===================WRITING TO SERIAL FOR float DATA TRANSMISSION:============================

  if (debug == 'N')
  {
    //-------GPS------

    Serial.write(*encGPS_lat);
    Serial.write(*encGPS_long);
    // ---- Accelerometer-----

    Serial.write(*encAccelX);

    Serial.write(*encAccelY);

    Serial.write(*encAccelZ);


    // ------Barometer------

    Serial.write(*encAlt);

    Serial.write(*encTemp);

    Serial.write(*encPress);

    char debug2 = 'N'; // debugg to see if serial.write worked
    if (debug2 == 'Y')
    {
      Serial.println(*encPress, DEC);

      while (Serial.available() > 0)
      {
        char inByte = Serial.read();
        Serial.println(inByte, HEX);
        Serial.println(*encPress);

      }
    }

  }





  delay(500);

}

//==================================FUNCTIONS:=================================

//-----------from last years code:------------
void Float2Byte(float f) { // Converts floats to bytes and writes them to serial
  byte * b = (byte *) &f; // pointer b points to f address (after it converts to byte) , then b is assigned to f data in byte form
  //  Serial.print("f:"); // data type
  Serial.write(b[0]);
  Serial.write(b[1]);
  Serial.write(b[2]);
  Serial.write(b[3]);


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
float AVG_Analog(float axisPin, int sampleSize)
{
  float reading = 0;

  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
  float AVG_Reading = reading / sampleSize;
  return AVG_Reading;

}

//Filters Data through a low pass filter. Use "alpha" to adjust filter strength.
float LowPassFilter(float OldVal, float NewRawVal, float alpha)
{
  float ProcessedVal;
  ProcessedVal = alpha * OldVal + (NewRawVal) * (1 - alpha);
  return ProcessedVal;
}
