// NOTE: '-----?' = check/look further into


#include <encoder_rrc_v2_4.h>
#include <Adafruit_BMP280.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <Wire.h>
#include <SPI.h>
#include <SD.h>


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
            ==>Wire.endTransmission(false) maybe gives a terminator-top bit which sends the previous Wire.write() buffer then sends a Start bit to initialize transmission once more.

        ==> Both Wire.write() and Wire.read()deals with writing/reading 1 byte at a time




*/
//*****************************************************************

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();



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

//#include <Adafruit_PMTK.h>

#define GPS Serial2 // RX, TX
//#define PMTK_SET_NMEA_UPDATERATE_1HZ "$PMTK220,1000*1F\r\n"
//#define PMTK_SET_NMEA_UPDATERATE_5HZ "$PMTK220,200*2C\r\n"
//#define PMTK_SET_NMEA_UPDATERATE_10HZ "$PMTK220,100*2F\r\n"
//#define isGPS_Beitian true // if using ground station "Beitian" gps =true

#define debug_GPS true

int pos = 0;
int stringplace = 0;
float GPS_latitude, GPS_longitude;
String nmea[15];
String labels[]={"RMC ID:\t ", "Time:\t ", "Data Validity (A=Y, V=N):\t ", "Latitude:\t ", "NS indicator:\t ", "Longitude:\t ", "EW indicator:\t ", "Speed:\t ", "Course over GND:\t","Date:\t","Mag variation:\t","Mag variation2:\t ","Pos Mode:\t","Nav Status:\t", "Checksum:\t","CR&LF:\t"};// in order of actual nmea code being read
//char PMTK_commands[55]; // max string length of PMTK command=50 + \r\n ~= 54 char total


//=================== 3. ADXL357Z=============================


//=================== 4.SD card==================================================
// --------sd variables to check sd info. (memory left, directory, make,etc.):-------
Sd2Card card;
SdVolume Volume;
SdFile root;
//------------------------------------------------------------------------------------
int sd_CSpin = BUILTIN_SDCARD;
char debug_SD = 'Y';
File dataFile;

//unsigned long timer = millis(); // uses millis function for timing
//int sdWrite_interval = 0; // sets interval time in count
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
  //analogReference(EXTERNAL); // use if power source isnt 3.3 or 5v
  Serial.begin(115200); // bmp280=9600 baud rate, MPU-AXL377 = 115200 baud rate
  bmp.begin(9600);


  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  while (Serial.available() == 0)
  { //doesnt do anything until detects buffer in serial ports
    Serial.println(F("Send byte to start data processing"));
    delay(5000); // repeat message every 5 seconds until a byte is sent
  }

  // =============== APPLY PMTK COMMANDS: ===============

 // GPS.printf(PMTK_SET_NMEA_UPDATERATE_1HZ);
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
  // initialize SD card:


  // see if the card is present and can be initialized:
  if (!SD.begin(sd_CSpin)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  else
  {
    Serial.println("card initialized.");
  }

  if (SD.exists("data.txt")) {
    Serial.println("data.txt exists.");
    dataFile = SD.open("data.txt");

    // read from the file until there's nothing else in it:
    if (debug_SD == 'Y') // only reads dataFile and writes to serial if debugging
    {
      Serial.println("#################### PRINTING SD card data from data.txt ###########################");
      while (dataFile.available()) {
        Serial.write(dataFile.read());
        delay(10);
      }
      Serial.println("#####################################################################################");

    }
    //  close the file then delete it :
    dataFile.close();
    SD.remove("data.txt");
  }


  else if (!SD.exists("data.txt")) {
    Serial.println("data.txt doesn't exist.");
  }
  delay(50);

  dataFile = SD.open("data.txt", FILE_WRITE);
  if (!dataFile) {
    Serial.println("error opening datalog.txt");
    while (1) ;
  }
  dataFile.println("count(t) at which data was written to file ,Latitude,Longitude,"
                   "AccelX (g),AccelY,AccelZ,"
                   "rawAccel X ,rawAccel Y,rawAccel Z,"
                   "Pressure (Pa),Temp (C), Alt (m)"); // header for txt file

  dataFile.close();
  delay(50);

  // ==============ADXL357z accelerometer:===================



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
  

  String GPS_message,GPS_ID;
  GPS.setTimeout(300); // in ms ; waits to read gps serial
  while (GPS.available() > 0)
  {
    GPS.readStringUntil('\n');
    /*
  GPS.read();
  if (GPS.find('$'))
   {
    GPS_ID = GPS.readStringUntil(',');

    
   }
   */
  }

 //if ((GPS_ID.substring(2,4)=="RMC")) { 
 if((GPS.find("$GNRMC"))||( GPS.find("$GPRMC")))
 {
    GPS_message = GPS.readStringUntil('\n');
    for (int i = 0; i < GPS_message.length(); i++) {
      if (GPS_message.substring(i, i + 1) == ",") {// if char in string=',' create a substring from i to i+1 position:
        nmea[pos] = GPS_message.substring(stringplace, i);
        stringplace = i + 1; //stringplace is used to get all characters within two "," thus used as another counter
        pos++;// indexing for nmea array==> basically index for substring
        // pos = substring(all characters per substring), i = entire string (all characters in string)
      }
      if (i == GPS_message.length() - 1) {
        nmea[pos] = GPS_message.substring(stringplace, i);
      }
    }
    // ===add to check if data = valid so if GPRMC contains "A":===

    //==============================================================
    GPS_latitude = (nmea[3].toFloat()) / 100;
    GPS_longitude = (nmea[5].toFloat()) / 100;

    if (debug_GPS == true)
    {
      Serial.print(labels[3]);

      Serial.print(GPS_latitude);
     Serial.println(nmea[4]);
      
      Serial.print(labels[5]);
      Serial.print(GPS_longitude);
      Serial.println(nmea[6]);
    }

  }
  else {
    if (debug_GPS == true)
    {
      Serial.print("No GPRMC NMEA code detected");
    }
  }

  stringplace = 0;
  pos = 0;




  //=========================== ADXL357z accelerometer:========================
  
  // =============Applying the encoder:===============


  int i;
  uint32_t int_data;
  uint8_t encGPS_lat[8], encGPS_long[8], encTemp[8], encPress[8], encAlt[8], encAccelX[8], encAccelY[8], encAccelZ[8];



  encode(GPS_latitude, 0x01, t, encGPS_lat);
  encode(GPS_longitude, 0x02, t, encGPS_long);
 /* encode(adjusted_gAccel_X, 0x03, t, encAccelX);
  encode(adjusted_gAccel_Y, 0x04, t, encAccelY);
  encode(adjusted_gAccel_Z, 0x05, t, encAccelZ);*/

  encode(bmp.readTemperature(), 0x06, t, encTemp);
  encode(bmp.readPressure(), 0x07, t, encPress);
  encode(bmp.readAltitude(), 0x00 , t, encAlt);

  //======Printing values===============================
  if (debug == 'Y')
  {
    //=========== GPS=====================
    Serial.println(F("\n****************** GPS: ********************"));

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



    // ADXL357z:
    Serial.println(F("\n****************** ADXL357z: ********************"));




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
  //Serial.println(millis());

  // Serial.println(t%3);
  //====================== writing to SD card:=================================================
  if ((t % 8) == 1 ) // prints to file every 8 intervals
  {
    //dataFile.print(millis());
    //dataFile.print(" ||  ");
    dataFile = SD.open("data.txt", FILE_WRITE);
    if (!dataFile) {
      Serial.println("error opening datalog.txt");
      while (1) ;
    }
    dataFile.print(t);
    dataFile.print(" ,  ");

    dataFile.print(GPS_latitude);
    dataFile.print(" ,  ");
    dataFile.print(GPS_longitude);
    dataFile.print(" ,  ");
    dataFile.print(bmp.readTemperature());
    dataFile.print(" ,  ");
    dataFile.print(bmp.readPressure());
    dataFile.print(" ,  ");
    dataFile.print(bmp.readAltitude());
    dataFile.println();
    dataFile.close();

  }
  //===========================================================================================
  t++;



  //===================WRITING TO SERIAL FOR float DATA TRANSMISSION:============================

  if (debug == 'N')
  {
    //-------GPS------

    Serial.write(*encGPS_lat);
    Serial.write(*encGPS_long);
    // ---- Accelerometer-----

   


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
