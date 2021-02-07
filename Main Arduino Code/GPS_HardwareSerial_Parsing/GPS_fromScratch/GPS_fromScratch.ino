#include <Adafruit_PMTK.h>
#include <string.h>
#include <stdio.h>
#define GPS Serial2 // RX, TX
#define PMTK_SET_NMEA_UPDATERATE_1HZ "$PMTK220,1000*1F\r\n"
#define PMTK_SET_NMEA_UPDATERATE_5HZ "$PMTK220,200*2C\r\n"
#define PMTK_SET_NMEA_UPDATERATE_10HZ "$PMTK220,100*2F\r\n"

int pos=0;
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
  GPS.printf(PMTK_SET_NMEA_UPDATERATE_1HZ);
  GPS.flush();

  // ====================================================
}

void loop() {
  

  while (GPS.available() > 0)
  {
    GPS.read();

  }
  
  if (GPS.find("$GPRMC,")) {
    String tempMsg = GPS.readStringUntil('\n');
    for (int i = 0; i < tempMsg.length(); i++) {
      if (tempMsg.substring(i, i + 1) == ",") {// if char in string=',' from i to i+1 position:
        nmea[pos] = tempMsg.substring(stringplace, i); 
        stringplace = i + 1; //stringplace is used to get all characters within two "," thus used as another counter
        pos++;// indexing for nmea array
      }
      if (i == tempMsg.length() - 1) {
        nmea[pos] = tempMsg.substring(stringplace, i);
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
}
