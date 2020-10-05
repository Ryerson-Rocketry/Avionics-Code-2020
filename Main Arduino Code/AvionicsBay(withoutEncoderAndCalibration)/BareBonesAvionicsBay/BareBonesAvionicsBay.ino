// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada



//=====================BMP280=========================
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C

//====================ADXL377=========================

// ====================GPS:===========================
#include <Adafruit_GPS.h>
//#include <SoftwareSerial.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

// you can change the pin numbers to match your wiring:
//SoftwareSerial mySerial(1, 0);
#define GPSSerial Serial1

Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

//uint32_t timer = millis();

//================================================
void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
//=========================GPS:===============================
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
 // GPS.sendCommand(PGCMD_ANTENNA);
// Ask for firmware version
  //mySerial.println(PMTK_Q_RELEASE);

//==============BMP280:======================

if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  
/* Default settings from datasheet. */
 
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode. 
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling //
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling 
                  Adafruit_BMP280::FILTER_X16,      // Filtering. //
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time. //

//=============ADXL377:======================  
  
 //========================================== 

  delay(500);
}

void loop() // run over and over again
{
  
  
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

   float GPSlat = GPS.latitude; 
  float GPSlon = GPS.longitude;

Serial.print(F("GPS DATA (lat,lon,#satillites,angle) respectfully are:   "));
Serial.print(GPSlat);
Serial.print(GPS.lat);
Serial.print(",     ");
Serial.print(GPSlon);
Serial.print(GPS.lon);
Serial.print(",     ");
Serial.print(GPS.satellites);
Serial.print(",     ");
Serial.print(GPS.angle);


 
  /*
    
    if (GPS.fix) {

      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
    
  */
//Serial.flush();
 
  //======================BMP280:==================
Serial.println("  ");
Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude()); // Adjusted to local forecast! 
    Serial.println(" m");

    Serial.println();
  
  //=====================ADXL377:==================
   
   unsigned int rawX = analogRead(A0);//analogRead(analogPinX);
  unsigned int rawY = analogRead(A1);
  unsigned int  rawZ = analogRead(A4);

  Serial.print(F("The raw Accel. values(X,Y,Z respectively) are:"));
  Serial.print(rawX);
  Serial.print(",  ");
  Serial.print(rawY);
  Serial.print(",  ");
  Serial.println(rawZ); 

  //===================WRITING TO SERIAL FOR ACTUAL DATA TRANSMISSION:============================

/*  
 //-------GPS------
Float2Byte(GPSlat);
Float2Byte(GPSlon);
Float2Byte(GPSspeed);
// ---- Accelerometer-----

// ------Barometer------


*/
 delay(500);
}

//-----------from last years code:------------ 
void Float2Byte(float f) { // Converts floats to bytes and writes them to serial
  byte * b = (byte *) &f; // pointer b points to f address (after it converts to byte) , then b is assigned to f data in byte form  
  //  Serial.print("f:"); // data type
  Serial.write(b[0]);
  Serial.write(b[1]);
  Serial.write(b[2]);
  Serial.write(b[3]);

  
}
