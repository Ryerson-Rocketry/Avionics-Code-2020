/*#######################MPU 6050 Accelerometer Library : Written by Taylor Schweizer #############################

==> from: https://learn-cnc.com/mpu-6050-example-i2c-primer/

###################################################################################################################
*/
#include<Wire.h>;

//The default address of the MPU is 0x68
const int MPU_addr = 0x68;
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
    register byte(think of like a specific address we want to read/write data to/from))(for reading: indicates the # of bytes to receive(Wire.requestFrom()),
    ACK/NACK bit,data byte 2(typically the actual data to write to the register), ACK/NACK bit, Stop bit]


      
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

  //According to the MPU 6050 Register Map, the PWR_MGMT_1 register allows us to configure the power mode of the MPU.
  //Looking at the register map page 40, PWR_MGMT_1 is located at 0x6B (107 in decimal, 01101011 in binary)
  //Think of each register in the MPU of having 8 bits that can be changed to change the settings.
  //For register 0x6B, the first three bits - Bit0 through Bit2 - are for selecting the clock source. A value of 0 sets the clock source to the internal 8MHZ oscillator, therefore the first three bits are all 0.
  //If we decided to use the PLL with external 19.2MHz reference, we would send 5 (decimal), so Bit0 would be 1, Bit1 would be 0, Bit2 would be 1, making 101 in binary which is 5 in decimal
  //Bit3 is labelled TEMP_DIS, and allows us to turn the temperature sensor on or off. A value of 1 turns if off, we might as well leave it on so Bit3 will be 0.
  //Bit4 is reserved, so we will just send 0 - the default
  //Bit5 is labelled CYCLE, which lets us specificy an automatic sleep cycle. We will set it to 0 and avoid the hassle of a sleep mode
  //Bit6 is labelled SLEEP, which lets us put it into sleep mode. Since we want the device on, we will set Bit6 to 0.
  //Bit7 is DEVICE_RESET, which resets all internal registers to their default values. We won't be doing that, so Bit7 is 0.

  //Based on the above, the 8 bits we will send to the MPU are all 0, so we can send any value of zero we want - hex 0x00, dec 0
  //This will power on the device and specify the initial power settings.
  //First, we tell the MPU which register we are writing to
  
  Wire.write(0x6B);// Wire.write actually puts the data in a buffer to send, the buffer is only released and sent when Wire.endTransmission is called

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
  // +++++++++++++++++++++++++Assigning MPU's max accel. : ++++++++++++++++++++++++++++


  Wire.beginTransmission(MPU_addr);

  //Next, let's do the accelerometer. I'll skip going through figuring out each bit, and just say I want to send 0x00 - there's almost no way we will experience more than 2g of acceleration on the wheelchair, so the full scale range can be +-2g. The address is 0x1C
  Wire.write(0x1C);

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

      

    


  //Once we send the values, we end the transmission because we don't have any further data to send for now
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
  
   The output of the MPU 6050 is 16 bits, but the I2C protocol demands 8 bit chunks. The registers are also composed of 8 bit chunks.
  You may notice that there is a register called ACCEL_XOUT_H and ACCEL_XOUT_L, as well as for the Y and Z axis. What is happening is that the data for the accelerometer is split into two separate registers.
  We need to combine the data from two registers to get the full value. Here's where some boolean operations start to be necessary.
  
  Anyways, since these registers are all 8 bit registers, but the values are 16 bit, we need to read two 8 bit numbers and combine them into a 16 bit number.
  If we start off with a 16 bit integer, and let it read the first 8 bits, then shift it by 8 bits and fill the rest in with the next register, it should be the value we want.
  */
  //First, lets create an 8 bit int to temporarily store the value
  byte XAxis = 0;
  //A byte is an 8 bit value. Assigning it to 0 means it is now 00000000

  //Let's go ahead and read the first register, ACCEL_XOUT_H
  Wire.beginTransmission(MPU_addr);
  //Tell the MPU that we want the data from register 59 (0x3B in hex)
  Wire.write(0x3B);
  //Send a Restart by setting endTransmission(false), which doesnt endTransmission,keeping the connection active, and allows for multiple consecutive writes: 
  // also sends the buffer data to mpu as data wont send with Wire.h without Wire.endTransmission call 
    // it may give a terminator-Stop bit which sends the previous Wire.write() buffer then sends a Start bit to initialize transmission once more.
   
  Wire.endTransmission(false);

  //Now we will reguest 1 register from the MPU. The third parameter 'true' says that we are ending the transmission after we get the data.
  
  Wire.requestFrom(MPU_addr, 1, true);
  //default is true and initiates after data is requested ==> this acts like Wire.endTransmision
  // but the reason why this is not used as a substitute to the above Wire.endTransmission(false
  // is because if it were to be done the data wont be sent from the buffer 
  
  //Now we will assign the byte from before to the value we are reading.
  XAxis = Wire.read();
  Wire.flush();  //We will now wait until we dont have data available

 
  //Let's repeat, and get the next value in the register.
  //NOTE: we really shouldn't be breaking up the measurements like this, since the value can change in an instant. But for education purposes, this should work fine.
  byte XAxisL = 0;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3C);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 1, true);
  XAxisL = Wire.read();
  Wire.flush();

  int16_t XAxisFull = XAxis << 8 | XAxisL;
  //The previous line of code bit shifts XAxis 8 places, then ORs it with XAxisL. Now, XAxisFull is the full binary value.


  float XAxisFinal = (float) XAxisFull*accelConfig_g/ maxAccel_raw;

  //+++++++++++++++++++++++++++ Y axis: +++++++++++++++++++++++++++++++++++++++++
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3D);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 2, true);
  int16_t YAxisFull =  (Wire.read() << 8 | Wire.read());
  Wire.flush();
  //++++++++++++++++++++++++++++ Z-axis: +++++++++++++++++++++++++++++++++++++++++
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3F);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 2, true);
 
  int16_t ZAxisFull =  (Wire.read() << 8 | Wire.read());
  
  Wire.flush();

    // convert raw accel. to g force:
  
  float YAxisFinal = (float)YAxisFull*accelConfig_g / maxAccel_raw;

  float ZAxisFinal = (float) ZAxisFull*accelConfig_g/ maxAccel_raw;
   // Question: what does the typecasting (float) above do to the BIN values?
      
   //Apply the LP filter: 
      
       float XAxisFinal_LPF = LowPassFilter(XAxisFinal_LPF,XAxisFinal,0.9);

       float YAxisFinal_LPF = LowPassFilter(YAxisFinal_LPF,YAxisFinal,0.9);

       float ZAxisFinal_LPF = LowPassFilter(ZAxisFinal_LPF,ZAxisFinal,0.01);

//++++++++++++++++++++++++++++ Printing accel. values: +++++++++++++++++++++++++++
  Serial.print("||raw X Axis (no LPF) =");
  Serial.print(XAxisFull);
  Serial.print("||raw Y Axis (no LPF) =");
  Serial.print(YAxisFull);
  Serial.print("||raw Z Axis (no LPF) =");
  Serial.println(ZAxisFull);

  
  Serial.print("||X Axis(g)(no LPF) =");
  Serial.print(XAxisFinal);
  Serial.print("||Y Axis(g) (no LPF) =");
  Serial.print(YAxisFinal);
  Serial.print("||Z Axis(g)(no LPF) =");
  Serial.println(ZAxisFinal);

  Serial.print("||X Axis(g)(w/ LPF) =");
  Serial.print(XAxisFinal_LPF);
  Serial.print("||Y Axis(g) (w/ LPF) =");
  Serial.print(YAxisFinal_LPF);
  Serial.print("||Z Axis(g)(w/ LPF) =");
  Serial.println(ZAxisFinal_LPF);
  
  // One could calculate the angle we are at knowing that when the sensor is completely level Z should be 1, if it isn't then we just take cos-1(Z) to get our angle with respect to the horizon. We can detect if we are moving
  //by extrapolating our angle and what X and Y should be, and then finding if there is any additional acceleration. But let's move on to the temperature sensor - use page 30 of the register map
//+++++++++++++++++++++++++++++++++++++++++ Gyro: +++++++++++++++++++++++++++++++++++++++
//  Wire.beginTransmission(MPU_addr);
//  Wire.write(0x43);
//  Wire.endTransmission(false);
//  Wire.requestFrom(MPU_addr, 6, true);
//  int16_t XGyroFull = Wire.read() << 8 | Wire.read();
//  int16_t YGyroFull = Wire.read() << 8 | Wire.read();
//  int16_t ZGyroFull = Wire.read() << 8 | Wire.read();
//  float XGyroFinal = (float)XGyroFull/32.8;
//  float YGyroFinal = (float)YGyroFull/32.8;
//  float ZGyroFinal = (float)ZGyroFull/32.8;
//  Serial.print("X Axis = ");
//  Serial.print(XGyroFinal);
//  Serial.println(" deg/s");
//  Serial.print("Y Axis = ");
//  Serial.print(YGyroFinal);
//  Serial.println(" deg/s");
//  Serial.print("Z Axis = ");
//  Serial.print(ZGyroFinal);
//  Serial.println(" deg/s");
  delay(500);
}
//Filters Data through a low pass filter. Use "alpha" to adjust filter strength.
float LowPassFilter(float OldVal, float NewRawVal, float alpha)
{
  float ProcessedVal;
  ProcessedVal = alpha * OldVal + (NewRawVal) * (1 - alpha);
  return ProcessedVal;
}
