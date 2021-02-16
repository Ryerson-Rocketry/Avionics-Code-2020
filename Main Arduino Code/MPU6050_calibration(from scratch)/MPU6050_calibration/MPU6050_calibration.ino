/*#######################MPU 6050 Accelerometer-REFERENCES: #############################

1. https://learn-cnc.com/mpu-6050-example-i2c-primer/

#########################################################################################
*/

#include<Wire.h>

#define MPU_addr  0x68 //The default address of the MPU is 0x68
#define register_powerManag 0x6B
#define register_accelConfig 0x1C
#define register_accelX 0x3B
#define register_accelY 0x3D
#define register_accelZ 0x3F


int maxAccel_raw = 32768; // raw output range = +-32 768
int accelConfig_g;// the config. set [+-(2g,4g,8g,or 16g)]
int gyroConfig; // the config. set +[+-(250,500,1000,or 2000)[in deg/s]]

void setup() {
  //Here we will begin the I2C communication
  Serial.begin(115200);

  Wire.begin();
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
  //Begin a serial connection so we can read the data
           
 // +++++++++++++++++++++++++Assigning MPU's max gyro: ++++++++++++++++++++++++++++

 //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



}

void loop() {
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
      
       float accelX_g_LPF = LowPassFilter(accelX_g_LPF,accelX_g,0.9);

       float accelY_g_LPF = LowPassFilter(accelY_g_LPF,accelY_g,0.9);

       //float accelZ_g_LPF = LowPassFilter(accelZ_g_LPF,accelZ_g,0.01);

//++++++++++++++++++++++++++++ Printing accel. values: +++++++++++++++++++++++++++
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
  
  // One could calculate the angle we are at knowing that when the sensor is completely level Z should be 1, if it isn't then we just take cos-1(Z) to get our angle with respect to the horizon. We can detect if we are moving
  //by extrapolating our angle and what X and Y should be, and then finding if there is any additional acceleration. But let's move on to the temperature sensor - use page 30 of the register map
//+++++++++++++++++++++++++++++++++++++++++ Gyro: +++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  delay(500);
}
//Filters Data through a low pass filter. Use "alpha" to adjust filter strength.
float LowPassFilter(float OldVal, float NewRawVal, float alpha)
{
  float ProcessedVal;
  ProcessedVal = alpha * OldVal + (NewRawVal) * (1 - alpha);
  return ProcessedVal;
}
