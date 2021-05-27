#include <Adafruit_PMTK.h>
#include <string.h>
#include <stdio.h>
#define GPS Serial4 // RX, TX
#define PMTK_SET_NMEA_UPDATERATE_1HZ "$PMTK220,1000*1F\r\n"
#define PMTK_SET_NMEA_UPDATERATE_5HZ "$PMTK220,200*2C\r\n"
#define PMTK_SET_NMEA_UPDATERATE_10HZ "$PMTK220,100*2F\r\n"
//#define isGPS_Beitian true // if using ground station "Beitian" gps =true
//#define debug_GPS false

int pos = 0;
int stringplace = 0;
float velocity;

String nmea[15];
String labels[12] {"Time: ", "Status: ", "Latitude: ", "Hemisphere: ", "Longitude: ", "Hemisphere: ", "Speed: ", "Track Angle: ", "Date: "};
//char PMTK_commands[55]; // max string length of PMTK command=50 + \r\n ~= 54 char total
void setup() {
  Serial.begin(115200);
  GPS.begin(9600);
  // =============== APPLY PMTK COMMANDS: ===============
  /* Serial.write is for sending BIN data, doesnt work with Serial.print but
    Serial.printf(formatted string, and in standard(stdio.h) lib & doesnt specify new line automatically) is for sending strings/chars
  */
  //strcpy(PMTK_commands,PMTK_SET_NMEA_UPDATE_1HZ);
  //strcat(PMTK_commands,"\r\n");
  //GPS.printf(PMTK_commands);
  //  GPS.printf(PMTK_SET_NMEA_UPDATERATE_1HZ);
  //GPS.flush();

  // ====================================================
}

void loop() {



  //Serial.println("VOID LOOP");
  String GPS_message;

  while (GPS.available() > 0)
  {

    GPS.read();
  }



  if ((GPS.find("$GPRMC,")) || (GPS.find("$GNRMC,"))) {
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

    //velocity = nmea[6].toFloat()* 0.514444; // knots to m/s conversion
    Serial.print(labels[2]);
    Serial.println(nmea[2].toFloat());
  }
  else {
    Serial.print("No GPRMC NMEA code detected");

  }


  stringplace = 0;
  pos = 0;

  delay(500);
}
