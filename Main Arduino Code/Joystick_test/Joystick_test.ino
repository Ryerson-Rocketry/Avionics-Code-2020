#define joyX A8
#define joyY A7
#define joyX2 A9
void setup() {
  Serial.begin(9600);
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


  delay(500);
}
