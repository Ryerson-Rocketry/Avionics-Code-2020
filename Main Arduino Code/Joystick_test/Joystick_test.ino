#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define joyX A8
#define joyY A7
#define joyX2 A9
int sd_CSpin = BUILTIN_SDCARD;
char debug_SD = 'Y';
File dataFile;
int t = 0;
void setup() {
  Serial.begin(9600);
  
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
    Serial.println("data.txt doesn't exist.");
        dataFile = SD.open("data.txt");

     while (dataFile.available()) {
        Serial.write(dataFile.read());
        delay(10);
      }
          dataFile.close();

  }   
  SD.remove("data.txt");

}
 
void loop() {
  // put your main code here, to run repeatedly:
  float xValue = analogRead(joyX);
    float xValue2 = analogRead(joyX2);

  float yValue = analogRead(joyY);
 
  //print the values with to plot or view
  Serial.print(xValue);
  Serial.print("\t");
    Serial.print(xValue2);
    Serial.print("\t");
    
  Serial.println(yValue);
    Serial.print("\t");
    if ((t % 5) == 1 ) // prints to file every 5 intervals
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

    dataFile.print(xValue);
    dataFile.print(" ,  ");
    dataFile.print(xValue2);
    dataFile.print(" ,  ");
    dataFile.print(yValue);
  
    dataFile.println();
    dataFile.close();

  }

t++; 
  delay(500);
}
