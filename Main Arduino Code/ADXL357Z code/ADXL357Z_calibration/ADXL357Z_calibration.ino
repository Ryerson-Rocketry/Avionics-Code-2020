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


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>


#define addr_ADXL357Z  0x1D //The default address of the MPU is 0x68
#define register_RESET 0x2F
#define register_STATUS 0x04
#define register_POWER_CTL 0x2D

#define register_RANGE 0x2C
#define register_accelX 0x0A
#define register_accelY 0x0D
#define register_accelZ 0x3F

//#define debug true
char debug = 'Y';
float sensitivity_scaleFactor = 78 * (10 ^ -6); // in g/LSB
float Min_rawY = 0, Max_rawY = 0, Min_rawZ = 2048, Max_rawZ = -2048, Min_rawX = 0, Max_rawX = 0;
int i = 0;


void setup() {
  Serial.begin(9600);

  while (Serial.available() == 0)
  {
    //doesnt do anything until detects buffer in serial ports
    Serial.println(F("Send byte to start data processing"));
    delay(5000);
  }
  Wire1.setSCL(37);
  Wire1.setSDA(38);
  Wire1.begin(addr_ADXL357Z);

 
    //==================== RESET EVERYTHING(if done see datasheet to double check ==>i believe it deletes some factory set data which is important)=============
 
   I2C_ACCEL(debug, addr_ADXL357Z, register_RESET, 'W', 0x52, 1);



  // =========== shut off temp processing:==============
  uint8_t powerControl_data[1];
  I2C_ACCEL(debug, addr_ADXL357Z, register_POWER_CTL, 'R', 0x00, 1);

  while (Wire1.available() > 0)
  {
    *powerControl_data = Wire1.read();
  }
  Serial.print("power control data is:\t");

  for (unsigned int ii = 0; ii < sizeof(powerControl_data) / sizeof(powerControl_data[0]); ii=ii+1)
  {
    Serial.print(powerControl_data[ii], BIN);
  }

  Serial.println("");




  const uint8_t TEMP_ON_OFF = 0x01; // off = 0000 0001, on =  0000 0000


  //     *powerControl_data = (*powerControl_data) | TEMP_ON_OFF;
  //I2C_ACCEL(debug, addr_ADXL357Z, register_POWER_CTL, 'W', *powerControl_data, 1,powerControl_data);
  Serial.print("TEMP_ON_OFF data is:\t");

  Serial.println(TEMP_ON_OFF, BIN);

  //Serial.print("new power control data is:\t");

 // Serial.println(*powerControl_data, BIN);







  Serial.println("");
  /*
      // ============= select accel range("in g") =====================

      I2C_ACCEL(debug, addr_ADXL357Z, register_RANGE, 'R', 0x00, 1);


      uint8_t rangeData[8];
      Serial.print("range Data is:\t");
      for (i = 0; i < 8; i++)
      {
        rangeData[i] = Wire.read();

        Serial.print(rangeData[i], BIN);
      }

      uint8_t I2C_FAST_FASTEST[] = {0x00, 0x80}; // i2c speed mode index=[0] fast and index=[1] fastest
      uint8_t accelRange[] = {0x01, 0x02, 0x03}; // accel range in g, order={10g,20g,40g}
      Serial.print("new range Data is: \t");
       rangeData = *rangeData| accelRange[2];

      for (i = 0; i < 8; i++)
      {
        Serial.print(rangeData[i], BIN);

      }
      I2C_ACCEL(debug, addr_ADXL357Z, register_RANGE, 'W', *rangeData, 1);

  */
  // ==== turn on measurement mode(power_ctl register):=============


}

void loop() {
  /*
    //========================check status ============================
    I2C_ACCEL(debug, addr_ADXL357Z, register_STATUS, 'R', 0x00, 1);
    uint8_t test_status,Status; int count=0;
    while (Wire.available()>0)
    {
    test_status= Wire.read();
    count= count+1;

    }
    Serial.print("counter is:\t");
    Serial.println(counter);
      Serial.print("size of status is:\t");
      Serial.println(sizeof(Status));

    Serial.print("status of adxl is:\t \n");

      Serial.print(Status, BIN);


      Serial.println("");


    Serial.println("");
    //=====================




    delay(1000);
  */


}
// functions:

int I2C_ACCEL(char debug, uint8_t address, uint8_t Register, char READ_OR_WRITE, uint8_t byte2write, int numbBytes2read)
{
  Wire1.beginTransmission(address);
  Wire1.write((byte)Register);
  if (READ_OR_WRITE == 'W')
  {
    Wire1.write((byte)byte2write);
    Wire1.endTransmission();

  }
  else if (READ_OR_WRITE == 'R')
  {
    Wire1.endTransmission(false);
    /*Sending a Restart by setting endTransmission(false), which doesnt endTransmission,keeping the connection active, and allows for multiple consecutive writes:
      also sends the buffer data to mpu as data wont send with Wire.h without Wire.endTransmission call
      it may give a terminator-Stop bit which sends the previous Wire.write() buffer then sends a Start bit to initialize transmission once more.
    */

    Wire1.requestFrom((int)address, numbBytes2read);
    /* default is true and initiates after data is requested ==> this acts like Wire.endTransmision
      but the reason why this is not used as a substitute to the above Wire.endTransmission(false
      is because if it were to be done the data wont be sent from the buffer */

  }
  // check to see status of transmission:
  if (debug == 'Y')
  {
    if ( Wire1.endTransmission(true) == 0)
    {
      Serial.println(F("\n TRANSMISSION SUCCESSFULL !!! [slave(ADXL357Z) sent ACK(Acknowledge) bit] "));
    }
    else
    {
      Serial.print(F("\n TRANSMISSION PROBLEM, Return is: \t"));
      Serial.println(Wire1.endTransmission(true));

    }
  }
  return 0;

}
