unsigned int rawX,rawY,rawZ;
void setup() {
Serial.begin(9600);

}  
int count=0; 

void loop() {

  rawX = analogRead(A0);
  rawY = analogRead(A1);
  rawZ = analogRead(A2);
  
 Serial.print(F("rawX:"));
 Serial.write(rawX);
 Serial.print(F("rawY:"));
 Serial.write(rawY);
 Serial.print(F("rawZ:"));
 Serial.write(rawZ);
 delay(500);


}
