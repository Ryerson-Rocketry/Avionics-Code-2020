  #include <Wire.h>

void setup(){
  Serial.begin(9600);
/*Start communication */
Wire.begin();
    // Put sensor as in Standby mode
    Wire.beginTransmission((byte)0x60); //0x60 is sensor address
    Wire.write((byte)0x26); //ctrl_reg
    Wire.write((byte)0x00); //reset_reg
    Wire.endTransmission();
    delay(10);
    // start sensor as Barometer Active
    Wire.beginTransmission((byte)0x60);
    Wire.write((byte)0x26); //ctrl_reg
    Wire.write((byte)0x01); //start sensor as barometer
    Wire.endTransmission();
    delay(10);
    }
void getdata(byte *a, byte *b, byte *c){
   Wire.beginTransmission(0x60); 
   Wire.write((byte)0x01);        // Data_PMSB_reg address
   Wire.endTransmission();    //Stop transmission
   Wire.requestFrom(0x60, 3); // "please send me the contents of your first three registers"
   //while(Wire.available()==0);
   *a = Wire.read(); // first received byte stored here
   *b = Wire.read(); // second received byte stored here
   *c = Wire.read(); // third received byte stored here
  }
void loop(){    
  byte aa,bb,cc;
  getdata(&aa,&bb,&cc);
  Serial.println(aa); //print aa for example
  Serial.println(bb); //print bb for example
  Serial.println(cc); //print cc for example
  delay(5000);
}
