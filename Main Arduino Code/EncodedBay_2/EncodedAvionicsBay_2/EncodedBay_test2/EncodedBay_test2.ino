#include <encoder_rrc_v2_4.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

//=====================BMP280=========================
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// bmp280 defaullt address is 0x77

//==================== MPU 6050: =========================
#define MPU_addr  0x68 //The default address of the MPU is 0x68
#define register_powerManag 0x6B
#define register_accelConfig 0x1C
#define register_accelX 0x3B
#define register_accelY 0x3D
#define register_accelZ 0x3F
int maxAccel_raw = 32768; // raw output range = +-32 768
int accelConfig_g;// the config. set [+-(2g,4g,8g,or 16g)]
int gyroConfig; // the config. set +[+-(250,500,1000,or 2000)[in deg/s]]
float Min_rawY=0,Max_rawY=0,Min_rawZ=2048,Max_rawZ=-2048,Min_rawX=0,Max_rawX=0;

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
              ==>Wire.endTransmission(false) maybe gives a terminator-Stop bit which sends the previous Wire.write() buffer then sends a Start bit to initialize transmission once more.

          ==> Both Wire.write() and Wire.read()deals with writing/reading 1 byte at a time

            
         
   
   */
//*****************************************************************
// ====================GPS:===========================
#include <Adafruit_GPS.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7
 /*
   ============================= NOTES ON GPS Protocols:===================
   
  a.  On NMEA protocols:
 https://www.gpsinformation.org/dale/nmea.htm
 https://en.wikipedia.org/wiki/NMEA_0183#:~:text=The%20NMEA%200183%20standard%20uses,%22listeners%22%20at%20a%20time.&text=The%20NMEA%20standard%20is%20proprietary,NMEA)%20as%20of%20November%202017.
b.  On geometric DOP(Dilution of Precision): https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
 
 Uses a specific serial interfacing protocol to communicate with microcontrollers called NMEA0183/NMEA2000.
 NMEA protocol consists of having 5 letters before the numerical data which defines the sentence: the 1st  two letters are GP, followed by 3 letters which indicate a specific sentence.
 Common Protocol fields are:
o GPGSA: lists the #satellites used for determining a fix position and gives a geometric DOP fix(Dilution of Precision; specifies the error propagation due to satellite geometry and positional measurement precision) ; uses 12 spaces for satellite numbers.  
o GPGSV: tells one about the satellites in view that it might be able to locate based on its viewing mask and almanac data; also tells one about the signal strength (SNR;signal to noise ratio);can only provide data to up to 4 satellites. 
o GPRMC: NMEA has its own version of essential gps pvt (position, velocity, time) data. It is called RMC, The Recommended Minimum
o GPGGA: essentially fix data which provides 3D location and its accuracy. 
*/ 


#define GPSSerial Serial2 //(serial2= TX-8,RX-7)

Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

//uint32_t timer = millis();

//================================================
void setup()
{
 /*
  "analogReference(EXTERNAL)":-------------?
  
    Tells the arduino that the reference volt value is not 3.3V or 5V, when using an 
    external batt source thats not 3.3 or 5v.
   */
   //analogReference(EXTERNAL); // use if power source isnt 3.3 or 5v--------?


  Serial.begin(9600);
//=========================GPS:===============================
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  bmp.begin(9600);

  Serial.println(F("Send byte to start data processing"));
  while(Serial.available()==0);//doesnt do anything until detects buffer in serial ports

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

//=============MPU6050:======================  
    Wire.begin();

  //Begin transmission with the MPU
  Wire.beginTransmission(MPU_addr);
 //+++++++++++++++Power Management:++++++++++++++++++++++++

/*----------------------  BITMAP:----------------------------------
 * 
 *  Bit0 through Bit2 - are for selecting the clock source
  Bit3 is labelled TEMP_DIS, and allows us to turn the temperature sensor on or off. A value of 1 turns if off, we might as well leave it on so Bit3 will be 0.
  Bit4 is reserved, so we will just send 0 - the default
  Bit5 is labelled CYCLE, which lets us specificy an automatic sleep cycle. We will set it to 0 and avoid the hassle of a sleep mode
  Bit6 is labelled SLEEP, which lets us put it into sleep mode. Since we want the device on, we will set Bit6 to 0.
  Bit7 is DEVICE_RESET, which resets all internal registers to their default values. We won't be doing that, so Bit7 is 0.

-------------------------------------------------------------------
*/
  
  Wire.write(register_powerManag);// Wire.write actually puts the data in a buffer to send, the buffer is only released and sent when Wire.endTransmission is called

  Wire.write(0x00);// set all bits in power regist. to 0 (turning on the mpu)
  
  //Now, I'm not sure if you can continue sending data by sending a new register address and then the data, or if you need to end the transmission first. Just to be on the safe side, we will end the transmission and restart it.
  Wire.endTransmission(true);
  if( Wire.endTransmission(true)==0)
      {
      Serial.println(F("MPU I2C-Power Management register-TRANSMISSION SUCCESSFULL (slave(mpu) sent ACK(Acknowledge) bit)!!! "));
      }
     else
        {
          Serial.print(F("MPU I2C:-Power Management register--TRANSMISSION PROBLEM, PLEASE LOOK INTO. NACK Returns:\t"));
          Serial.println(Wire.endTransmission(true));
          
        }
  // changing MPU's clk from 8hz internal clk to gyroX clk as the datasheet says its recommended for stability purposes:
  uint8_t powerManag_data=0;
  uint8_t mpuCLK_array[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};

      powerManag_data = powerManag_data|mpuCLK_array[1]; //selected gyroX clk 
  
     Wire.beginTransmission(MPU_addr);

    Wire.write(register_powerManag);// Wire.write actually puts the data in a buffer to send, the buffer is only released and sent when Wire.endTransmission is called

  Wire.write(powerManag_data);// set all bits in power regist. to 0 (turning on the mpu)
  
  //Now, I'm not sure if you can continue sending data by sending a new register address and then the data, or if you need to end the transmission first. Just to be on the safe side, we will end the transmission and restart it.
  Wire.endTransmission(true);
  if( Wire.endTransmission(true)==0)
      {
      Serial.println(F("MPU I2C-Power Management register-TRANSMISSION SUCCESSFULL (slave(mpu) sent ACK(Acknowledge) bit)!!! "));
      }
     else
        {
          Serial.print(F("MPU I2C:-Power Management register--TRANSMISSION PROBLEM, PLEASE LOOK INTO. NACK Returns:\t"));
          Serial.println(Wire.endTransmission(true));
          
        }

    // +++++++++++++++++++++++++Assigning MPU's max accel. : ++++++++++++++++++++++++++++


  Wire.beginTransmission(MPU_addr);

  //Next, let's do the accelerometer. I'll skip going through figuring out each bit, and just say I want to send 0x00 - there's almost no way we will experience more than 2g of acceleration on the wheelchair, so the full scale range can be +-2g. The address is 0x1C
  Wire.write(register_accelConfig);

  const uint8_t accelConfigArray[] = {0x00,0x08,0x10,0x18};// array order is: {2g,4g,8g,16g}

  const int accelConfigArray_g [] = {2,4,8,16};
 
  uint8_t accelConfig = accelConfigArray[3];// config value that will be written to mpu
  
  Wire.write(accelConfig); 

  Serial.print(F("Size of accelConfigArray_g is:\t"));
  Serial.println(sizeof(*accelConfigArray_g));

  
  Serial.print(F("Size of accelConfigArray is:\t"));
  Serial.println(sizeof(*accelConfigArray));
  
  // assigning the accel config value in g's:
  
        for (int counter=0; counter<4; counter++)
            {
            if (accelConfig == accelConfigArray[counter])
              {
                accelConfig_g = accelConfigArray_g[counter];
              }
            }
    
    Serial.print("accel. config (in g's) is:\t");
    Serial.println(accelConfig_g);

  //Wire.endTransmission(true);
  // checking if transmission received:
  
    Serial.print(F("# of accelConfig. Transmission bytes received:\t"));
    Serial.println(Wire.requestFrom(MPU_addr,1,true));
    if( Wire.endTransmission(true)==0)
      {
      Serial.println(F("MPU I2C-Accel Config register-TRANSMISSION SUCCESSFULL (slave(mpu) sent ACK(Acknowledge) bit)!!! "));
      }
     else
        {
          Serial.print(F("MPU I2C:-Accel. Config register-TRANSMISSION PROBLEM, PLEASE LOOK INTO. NACK Returns:\t"));
          Serial.println(Wire.endTransmission(true));
          
        }
           
 // +++++++++++++++++++++++++Assigning MPU's max gyro: ++++++++++++++++++++++++++++

 //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

if (GPS.fix)
{
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
}

 

 
  //======================BMP280:==================
  
  
  //===================== MPU6050 : ===============================================
   
//++++++++++++++++++++ X-axis: +++++++++++++++++++++++++++++++++++++++++++++

  /*
    the MPU stores its data into 2 8 bit chunks therefore we will combine them into a 16bit variable by shifting bits to the left by 8 bits then OR-ing the shifted (previously read bits w/ the incoming new bits)
    this will tell us exactly what value the mpu's outputting. 
  */
  

  Wire.beginTransmission(MPU_addr);
  Wire.write(register_accelX);
  
  Wire.endTransmission(false);
     /*Sending a Restart by setting endTransmission(false), which doesnt endTransmission,keeping the connection active, and allows for multiple consecutive writes: 
  also sends the buffer data to mpu as data wont send with Wire.h without Wire.endTransmission call 
  it may give a terminator-Stop bit which sends the previous Wire.write() buffer then sends a Start bit to initialize transmission once more. */
 
  Wire.requestFrom(MPU_addr, 2, true);
    //default is true and initiates after data is requested ==> this acts like Wire.endTransmision
  // but the reason why this is not used as a substitute to the above Wire.endTransmission(false
  // is because if it were to be done the data wont be sent from the buffer 
  

  int16_t accelX_16bit = Wire.read() << 8 | Wire.read();

  Wire.flush();


  float accelX_g = (float) accelX_16bit*accelConfig_g/ maxAccel_raw; // typecasting to make accelX_16bit a float

  //+++++++++++++++++++++++++++ Y axis: +++++++++++++++++++++++++++++++++++++++++
  Wire.beginTransmission(MPU_addr);
  Wire.write(register_accelY);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 2, true);
  int16_t accelY_16bit =  (Wire.read() << 8 | Wire.read());
  Wire.flush();
  //++++++++++++++++++++++++++++ Z-axis: +++++++++++++++++++++++++++++++++++++++++
  Wire.beginTransmission(MPU_addr);
  Wire.write(register_accelZ);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 2, true);
 
  int16_t accelZ_16bit =  (Wire.read() << 8 | Wire.read());
  
  Wire.flush();

    // convert raw accel. to g force:
  
  float accelY_g = (float)accelY_16bit*accelConfig_g / maxAccel_raw;

  float accelZ_g = (float) accelZ_16bit*accelConfig_g/ maxAccel_raw;
      
   //Apply the LP filter: 
      
       float accelX_g_LPF=0;
       accelX_g_LPF = LowPassFilter(accelX_g_LPF,accelX_g,0.9);

       float accelY_g_LPF=0;
       accelY_g_LPF= LowPassFilter(accelY_g_LPF,accelY_g,0.9);

       //float accelZ_g_LPF = LowPassFilter(accelZ_g_LPF,accelZ_g,0.01);

//++++++++++++++++++++++++++++ Printing accel. values: +++++++++++++++++++++++++++
    //---------Determining max and min raw accel. values:---------

  if ((float)accelX_16bit < Min_rawX)
  {
    Min_rawX = (float)accelX_16bit;
  }
  if ((float)accelX_16bit > Max_rawX)
  {
    Max_rawX = (float)accelX_16bit;
  }
  if ((float)accelY_16bit < Min_rawY)
  {
    Min_rawY = (float)accelY_16bit;
  }
  if ((float)accelY_16bit > Max_rawY)
  {
    Max_rawY = (float)accelY_16bit;
  }
  if ((float)accelZ_16bit < Min_rawZ)
  {
    Min_rawZ = (float)accelZ_16bit;
  }
  if ((float)accelZ_16bit > Max_rawZ)
  {
    Max_rawZ = (float)accelZ_16bit;
  }
  //--------------------------------------------
  Serial.print("||raw X Axis (no LPF) =");
  Serial.print(accelX_16bit);
  Serial.print("||raw Y Axis (no LPF) =");
  Serial.print(accelY_16bit);
  Serial.print("||raw Z Axis (no LPF) =");
  Serial.println(accelZ_16bit);

  /*
  Serial.print("||X Axis(g)(no LPF) =");
  Serial.print(accelX_g);
  Serial.print("||Y Axis(g) (no LPF) =");
  Serial.print(accelY_g);
  Serial.print("||Z Axis(g)(no LPF) =");
  Serial.println(accelZ_g);
*/
  Serial.print("||X Axis(g)(w/ LPF) =");
  Serial.print(accelX_g_LPF);
  Serial.print("||Y Axis(g) (w/ LPF) =");
  Serial.print(accelY_g_LPF);
  Serial.print("||Z Axis(g)(no LPF) =");
  Serial.println(accelZ_g);
  
  Serial.print(F("min and max rawX,Y,Z value respectively are:" ));  
  Serial.print(Min_rawX);
      Serial.print("-" );  
  Serial.print(Max_rawX);
  Serial.print(",  ");
  Serial.print(Min_rawY);
      Serial.print("-" );  
  Serial.print(Max_rawY);
  Serial.print(",  ");
  Serial.print(Min_rawZ);
      Serial.print("-" );  
  Serial.println(Max_rawZ);
  
  // One could calculate the angle we are at knowing that when the sensor is completely level Z should be 1, if it isn't then we just take cos-1(Z) to get our angle with respect to the horizon. We can detect if we are moving
  //by extrapolating our angle and what X and Y should be, and then finding if there is any additional acceleration. But let's move on to the temperature sensor - use page 30 of the register map
//+++++++++++++++++++++++++++++++++++++++++ MPU6050 Gyro: +++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++Applying the encoder:++++++++++++++++++++++++++++++++++++++++++++


  int i,t=0;
  uint32_t int_data;
  uint8_t encGPS_lat[8],encGPS_long[8],encpoten[8],encTemp[8],encPress[8],encAlt[8],encAccelX[8],encAccelY[8],encAccelZ[8];
  
  

//Adafruit GPS:
 

   Serial.print(F(" GPS lat:  "));

        Serial.print(GPS.latitude);
  Serial.print(F(" = "));
  encode(GPS.latitude, 0x01, t, encGPS_lat);
   for (i=0;i<8;i++)
  {
    Serial.print(encGPS_lat[i],HEX);
  }
 Serial.println(F(" GPS long:  "));

   // Serial.print(temp);
        Serial.print(GPS.lon);
  Serial.print(F(" = "));
  encode(GPS.longitude, 0x02, t, encGPS_long);
   for (i=0;i<8;i++)
  {
    Serial.print(encGPS_long[i],HEX);
  }

 
 //bmp280 temperature:
  Serial.println();
      Serial.print(F("bmp temp(C):  "));
      //temp = bmp.readTemperature();
   // Serial.print(temp);
        Serial.print(bmp.readTemperature());
  Serial.print(F(" = "));
  encode(bmp.readTemperature(), 0x06, t, encTemp);
   for (i=0;i<8;i++)
  {
    Serial.print(encTemp[i],HEX);
  }
  Serial.println();

  // bmp pressure;
      Serial.print(F("bmp pressure(Pa):  "));
    Serial.print(bmp.readPressure());
  Serial.print(F(" = "));
  encode(bmp.readPressure(),0x07, t, encPress);
  
  for (i=0;i<8;i++)
  {
    Serial.print(encPress[i],HEX);
  }
  
  Serial.println();
  
  //bmp altitude:
  
  Serial.print(F("bmp alt(m):  "));
  Serial.print(bmp.readAltitude(1013.25));// 1013.25 is SL pressure in hPa
  Serial.print(F(" = "));
  encode(bmp.readAltitude(1013.25),0x00 , t, encAlt);
  

// printing the encoder values, bit by bit: 
  for (i=0;i<8;i++)
  {
    Serial.print(encAlt[i],HEX);
  
  }
  
  Serial.println();


// printing the encoder values, bit by bit: 
 
 
 //MPU6050:   
    Serial.print(F("scaledX acceleration(g):  "));
    Serial.print(accelX_g_LPF);
  Serial.print(" = ");
  encode(accelX_g_LPF,0x03, t, encAccelX);
   for (i=0;i<8;i++)
  {
    Serial.print(encAccelX[i],HEX);
  }
 
        Serial.println(F(""));
    Serial.print(F("scaledY acceleration(g):  "));
    Serial.print(accelY_g_LPF);
  Serial.print(" = ");
  encode(accelY_g_LPF,0x04, t, encAccelY);
   for (i=0;i<8;i++)
  {
    Serial.print(encAccelY[i],HEX);
  }
      Serial.println(F(""));

 
    Serial.print(F("scaledZ acceleration(g):  "));
    Serial.print(accelZ_g);
  Serial.print(F(" = "));
  encode(accelZ_g,0x05, t, encAccelZ);
   for (i=0;i<8;i++)
  {
    Serial.print(encAccelZ[i],HEX);
  }


  
  //===================WRITING TO SERIAL FOR float DATA TRANSMISSION:============================

/*  
 //-------GPS------
Float2Byte(GPSlat);
Float2Byte(GPSlon);
Float2Byte(GPSspeed);
// ---- Accelerometer-----

// ------Barometer------


*/
        t++;

 delay(2000);// wait for 2 sec for next data sets
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

//Filters Data through a low pass filter. Use "alpha" to adjust filter strength.
float LowPassFilter(float OldVal, float NewRawVal, float alpha)
{
  float ProcessedVal;
  ProcessedVal = alpha * OldVal + (NewRawVal) * (1 - alpha);
  return ProcessedVal;
}
