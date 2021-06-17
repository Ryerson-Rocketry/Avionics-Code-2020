#include <Adafruit_PMTK.h>
#include <string.h>
#include <stdio.h>

#define GPS Serial2 // RX, TX
//#define PMTK_SET_NMEA_UPDATERATE_1HZ "$PMTK220,1000*1F\r\n"
//#define PMTK_SET_NMEA_UPDATERATE_5HZ "$PMTK220,200*2C\r\n"
//#define PMTK_SET_NMEA_UPDATERATE_10HZ "$PMTK220,100*2F\r\n"

#define debug_GPS true

int pos = 0;
int stringplace = 0;
float  Latitude, Longitude;

String nmea[15];
String labels[] = {"RMC ID:\t ", "Time:\t ", "Data Validity (A=Y, V=N):\t ", "Latitude:\t ", "NS indicator:\t ", "Longitude:\t ", "EW indicator:\t ", "Speed:\t ", "Course over GND:\t", "Date:\t", "Mag variation:\t", "Mag variation2:\t ", "Pos Mode:\t", "Nav Status:\t", "Checksum:\t", "CR&LF:\t"}; // in order of actual nmea code being read
//char PMTK_commands[55]; // max string length of PMTK command=50 + \r\n ~= 54 char total
void setup() {
  Serial.begin(115200);
  GPS.begin(9600);
  // =============== APPLY PMTK COMMANDS: ===============

  // ====================================================
}

void loop() {



  String GPS_message, GPS_ID;



  if (GPS.find("RMC") )
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
    // ===add to check if data = valid so if GPRMC contains ",A,":===

    //==============================================================
    Latitude = (nmea[3].toFloat()) / 100;
    Longitude = (nmea[5].toFloat()) / 100;

    if (debug_GPS == true)
    {
      Serial.print(labels[3]);

      Serial.print(Latitude);
      Serial.println(nmea[4]);

      Serial.print(labels[5]);
      Serial.print(Longitude);
      Serial.println(nmea[6]);
    }

  }
  else
  {
    if (debug_GPS == true)
    {
      Serial.println("No GPRMC NMEA code detected");
    }
  }



  stringplace = 0;
  pos = 0;

}
