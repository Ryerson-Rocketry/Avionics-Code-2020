#include <rrc_encoder.h>
//#include <adxl357.h>
#include <Adafruit_BMP280.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>


#define GPS Serial2 // RX, TX
#define debug_GPS false


// NOTE: '-----?' = check/look further into


// ***************************** DEFINING GLOBAL VARIABLES:****************************************

//==================== 1. BMP280=================================
Adafruit_BMP280 bmp; // use I2C interface
//Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
//Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

float BMP_temp, BMP_press, BMP_alt;
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





// ==================== 2. GPS:===========================


/*
  ============================= GPS Protocols:===================

  -------------------------------------references:-----------------------------------
  a.  On NMEA protocols:
   https://www.gpsinformation.org/dale/nmea.htm
   https://en.wikipedia.org/wiki/NMEA_0183#:~:text=The%20NMEA%200183%20standard%20uses,%22listeners%22%20at%20a%20time.&text=The%20NMEA%20standard%20is%20proprietary,NMEA)%20as%20of%20November%202017.
  b.  On geometric DOP(Dilution of Precision): https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
  ------------------------------------NOTES: --------------------------------------------------------------------------------------

   Uses a specific serial interfacing protocol to communicate with microcontrollers called NMEA0183/NMEA2000.
   NMEA protocol consists of having 5 letters before the numerical data which defines the sentence: the 1st  two letters are GP, followed by 3 letters which indicate a specific sentence.
   Common Protocol fields are:
  o GPGSA: lists the #satellites used for determining a fix position and gives a geometric DOP fix(Dilution of Precision; specifies the error propagation due to satellite geometry and positional measurement precision) ; uses 12 spaces for satellite numbers.
  o GPGSV: tells one about the satellites in view that it might be able to locate based on its viewing mask and almanac data; also tells one about the signal strength (SNR;signal to noise ratio);can only provide data to up to 4 satellites.
  o GPRMC: NMEA has its own version of essential gps pvt (position, velocity, time) data. It is called RMC, The Recommended Minimum
  o GPGGA: essentially fix data which provides 3D location and its accuracy.

*/




int pos = 0;
int stringplace = 0;
float GPS_latitude, GPS_longitude;
String nmea[15];
String labels[] = {"RMC ID:\t ", "Time:\t ", "Data Validity (A=Y, V=N):\t ", "Latitude:\t ", "NS indicator:\t ", "Longitude:\t ", "EW indicator:\t ", "Speed:\t ", "Course over GND:\t", "Date:\t", "Mag variation:\t", "Mag variation2:\t ", "Pos Mode:\t", "Nav Status:\t", "Checksum:\t", "CR&LF:\t"}; // in order of actual nmea code being read
//char PMTK_commands[55]; // max string length of PMTK command=50 + \r\n ~= 54 char total


//=================== 3. ADXL357Z=============================


//=================== 4.SD card==================================================
// --------sd variables to check sd info. (memory left, directory, make,etc.):-------
Sd2Card card;
SdVolume Volume;
SdFile root;
int sd_CSpin = BUILTIN_SDCARD;
char debug_SD = 'Y';
File dataFile;

//int sdWrite_interval = 0; // sets interval time in count
//====================================================================================

//============================= Shared Global Variables(Global Variables used by all sensors): =============================================

char debug = 'Y';// 'Y' if debugging and printing all testing values to serial monitor
int t = 0;
//==========================================================================================================================================


void setup() {


  Serial.begin(115200); // bmp280=9600 baud rate, MPU-AXL377 = 115200 baud rate
  bmp.begin(9600);


  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  while (!Serial) {

    ; // wait for serial port to connect.
  }

  //Serial.setTimeout(300);
  // =============== APPLY PMTK COMMANDS: ===============

  // GPS.printf(PMTK_SET_NMEA_UPDATERATE_1HZ);
  //=======================================================

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
  //======================================================

  //======================initialize SD card:========================


  // see if the card is present and can be initialized:
  if (SD.begin(sd_CSpin))
  {
    Serial.println("card initialized.");
  }


  else if (!SD.begin(sd_CSpin)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }


  if (SD.exists("data.txt")) {
    Serial.println("data.txt exists.");
    dataFile = SD.open("data.txt", FILE_READ);

    // read from the file until there's nothing else in it:
    if (debug_SD == 'Y') // only reads dataFile and writes to serial if debugging
    {
      Serial.println("#################### PRINTING SD card data from data.txt ###########################");
      while (dataFile.available()) {
        Serial.write(dataFile.read());
      }
      Serial.println("#####################################################################################");

    }
    //  close the file then delete it :
    dataFile.close();
    SD.remove("data.txt");
  }


  else  {
    Serial.println("data.txt doesn't exist.");
  }

  //==============================================================

  // ==============ADXL357z accelerometer:===================

  //===========================================================


}

void loop() {


  //===========================GPS:=======================




  String GPS_message, GPS_ID;



  if (GPS.find("RMC"))
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

  //================================================================================

  //====================== BMP280: ======================================

  BMP_temp = bmp.readTemperature();
  BMP_press = bmp.readPressure();
  BMP_alt = bmp.readAltitude();



  //=====================================================================


  //=========================== ADXL357z accelerometer:========================


  //===========================================================================

  // =============Applying the encoder:===============


  int i;
  uint32_t int_data;
  uint8_t encGPS_lat[8], encGPS_long[8], encTemp[8], encPress[8], encAlt[8], encAccelX[8], encAccelY[8], encAccelZ[8];



  encode(GPS_latitude, 0x01, t, encGPS_lat);
  encode(GPS_longitude, 0x02, t, encGPS_long);


  encode(BMP_temp, 0x06, t, encTemp);
  encode(BMP_press, 0x07, t, encPress);
  encode(BMP_alt, 0x00 , t, encAlt);

  //======Printing values===============================
  if (debug == 'Y')
  {
    //=========== 1.GPS=====================
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



    // 2. ADXL357z:
    Serial.println(F("\n****************** ADXL357z: ********************"));




    //3.bmp280 temperature:

    Serial.println(F("****************** BMP280: ********************"));
    Serial.print(F("bmp temp(C):  "));

    Serial.print(BMP_temp);
    Serial.print(F(" = "));
    for (i = 0; i < 8; i++)
    {
      Serial.print(encTemp[i], HEX);
    }
    Serial.println();


    //4. bmp pressure;

    Serial.print(F("bmp pressure(Pa):  "));
    Serial.print(BMP_press);
    Serial.print(F(" = "));
    for (i = 0; i < 8; i++)
    {
      Serial.print(encPress[i], HEX);
    }
    Serial.println();

    //5.bmp altitude:

    Serial.print(F("bmp alt(m):  "));
    Serial.print(BMP_alt);// 1013.25 is SL pressure in hPa
    Serial.print(F(" = "));



    // printing the encoder values, bit by bit:
    for (i = 0; i < 8; i++)
    {
      Serial.print(encAlt[i], HEX);

    }



    Serial.print("\n t is:\t");
    Serial.println(t);

  }

  //====================== writing to SD card:=================================================

  String sensorString[] = {"Time: ", "Lat: ", "Long:  ", "Temp(degC):  ", "Pressure(Pa):  ", "Alt(m):  "};
  float sensorArray[6] = {float(t), GPS_latitude, GPS_longitude, BMP_temp, BMP_press, BMP_alt};
  if ((t % 8) == 1 ) // prints to file every 8 intervals
  {

    dataFile = SD.open("data.txt", FILE_WRITE);
    if (dataFile)
    {

      for (int count = 0; count < 6; count++)
      {
        sensorString[count] += String(sensorArray[count]);
        dataFile.print(sensorString[count]);
      }

      //dataFile.flush();

      dataFile.close();
    }
    else
    {
      Serial.println("ERROR OPENING data.txt file");
    }
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


//Filters Data through a low pass filter. Use "alpha" to adjust filter strength.
float LowPassFilter(float OldVal, float NewRawVal, float alpha)
{
  float ProcessedVal;
  ProcessedVal = alpha * OldVal + (NewRawVal) * (1 - alpha);
  return ProcessedVal;
}
