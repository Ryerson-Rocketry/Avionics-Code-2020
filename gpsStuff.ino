#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
void setup() {
    Serial.begin(9600);
    GPS.begin(9600);
}

void loop() {
    while (Serial.available()){
        Serial.print(char(Serial.read()));    
    }
}
