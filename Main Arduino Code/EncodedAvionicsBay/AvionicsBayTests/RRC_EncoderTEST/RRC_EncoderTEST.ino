/*
 * ==============REFERENCES:=================
 1. floating-point encoding & float to byte equations:
  a. General stuff:
  i. https://stackoverflow.com/questions/48231407/byte-representation-of-float-as-integer-equation
  ii. https://en.wikipedia.org/wiki/IEEE_754
  iii. https://www.oreilly.com/library/view/c-in-a/0596006977/ch04.html
  
  b. about overflow and encoding format: 
    i. https://stackoverflow.com/questions/20633163/how-can-a-4-byte-store-such-large-value-of-3-4e38/20633632#20633632
    ii. https://stackoverflow.com/questions/14001910/what-is-this-denormal-data-about-c/14002396#14002396
 */


#ifndef ENCODER_RRC_H
#define ENCODER_RRC_H

#define BAR 0x00                  //  barometer altitude
#define TEMP 0x06 // barometer temperature
#define PRESS 0x00 // barometer pressure
#define GPS_L 0x01                //  gps longitude
#define GPS_T 0x02                //  gps latitude
#define ACC_X 0x03                //  accelerometer x
#define ACC_Y 0x04                //  accelerometer y
#define ACC_Z 0x05                //  accelerometer z


#define BYTES 6
#define C_SHIFT 1
#define H_SHIFT 5
#define P_SHIFT 5
#define P_OFFSET 6
#define PORTION 0x000007c0

#define TIME_STAMP 3
#define T_PORTION 0x001f
#define T_SHIFT 5

void setup()
{
  Serial.begin(9600);
}

// reasoning behind the shifts==>  shift results to the lowest bits so it fits in the 8bit integer type (int8_t). 
// LSB of byte 3, MSB of byte 2, MSB of byte 1, LSB of byte 1
uint8_t _checksum(uint32_t data)
{
  uint8_t result = 0;
  Serial.print("1st, checksum result is:" );
  Serial.print(result);
  result |= 0x01 & (data & 0x01);     //  LSB
  
  Serial.print(",   2nd, checksum result is:" );
  Serial.print(result);
  result |= 0x02 & ((data & 0x80) >> 6);              //  MSB of the lowest byte
  
  Serial.print(",    3rd, checksum result is:" );
  Serial.print(result);
  result |= 0x04 & ((data & 0x8000) >> 13);           //  MSB of the middle byte
  
  Serial.print(",   4th, checksum result is:" );
  Serial.print(result);
  result |= 0x08 & ((data & 0x10000) >> 13);          //  LSB of the higest byte
  
  Serial.print(",   5th, checksum result is:" );
  Serial.println(result);
  return result;
}


//  convert the float data to 3 byte int and flip the higest bit if negative
uint32_t _float2int(float data, int range)
{
  bool neg = false;
  if(data < 0)
  {
    neg = true;
    data *= -1; //  I don't want usual 2's complement
    Serial.print("1's compliment data is:");
    Serial.print(data);
  }
  uint32_t int_data = data*range+0.5;
  if(neg == true) int_data |= 0x800000;
  //  flip higest bit
  Serial.print(",   the range is:  ");
  
  Serial.print(range);
Serial.print(",    1's compliment data*range +0.5 is:");
    Serial.println(int_data);
  return int_data;
}


//  encodes the float into bytes with headers and checksum
int encode(float data, int header, uint16_t time, uint8_t *out)
{
  uint32_t int_data;
  if(header == BAR) int_data = _float2int(data, 10);  //  barometer only needs 2 decimal points
  else int_data = _float2int(data, 10000);  //  everything else needs 4 decimal points
  
  //-------------------stuff added to original code:----------------
if(header == TEMP) int_data = _float2int(data, 10);  //  barometer only needs 2 decimal points
  else int_data = _float2int(data, 10000);            //  everything else needs 4 decimal points
if(header == PRESS) int_data = _float2int(data, 10);  //  barometer only needs 2 decimal points
  else int_data = _float2int(data, 10000);            //  everything else needs 4 decimal points

//-------------------------------------------------------------------

  uint8_t checks = _checksum(int_data) << C_SHIFT;    //  generate the checksum before data manipulation
  int_data <<= 8;                                     //  pad it to the right to get all bit needed with the same ORing portion
  int portion = PORTION;

  for(int i = 0; i < BYTES; i++)
  {
    out[BYTES-1-i] = 0;                             //  init as 0
    out[BYTES-1-i] |= header << H_SHIFT;            //  add header after shifting it to its place
    out[BYTES-1-i] |= (int_data & portion) >> i*P_SHIFT+P_OFFSET;  //  add the portion of data after shift to remove zeros
    portion <<= P_SHIFT;                            //  shift the portion for next iteration
  }

  out[0] |= checks & 0x0f << C_SHIFT;                 //  add checksum to the first byte (0x1e = 0001 1110)

  int t_portion = T_PORTION;
  for(int i = 0; i < TIME_STAMP; i++)                 //  add time stamp
  {
    out[BYTES+TIME_STAMP-2-i] |= header << H_SHIFT; //  add header after shifting it to its place
    out[BYTES+TIME_STAMP-2-i] |= (time & t_portion) >> i*T_SHIFT;  //  add the portion after shifting it 
    t_portion <<= T_SHIFT;                          //  shift the portion for next iteration
  }

  return 0;
}

#endif

void loop()
{
  float SensorData =20.3; 
  int SensorHeader=BAR,i;
  uint16_t UART_time = 5; //millis();
  //Serial.print("time since uart transmission started is:"); 
  //Serial.println(UART_Time);
  int Size_EncodedData =8;
  uint8_t EncodedData[Size_EncodedData];
  Serial.print("The data,header, and time sent to encode is:");
  Serial.print("  ");
  Serial.print(SensorData);
  Serial.print(",  ");
    Serial.print(SensorHeader);
  Serial.print(",  ");
 Serial.println(UART_time);
    Serial.print(",    in binary = ");
    Serial.print(SensorData,BIN);
 Serial.print(",  ");
 Serial.print(SensorHeader,BIN);
 Serial.print(",  ");
 Serial.println(UART_time,BIN);

 
        Serial.print("The encoded Data: ");

  
  
  encode(SensorData, SensorHeader,UART_time, EncodedData);

for (i=0;i<8;i++)
  {
    Serial.print(EncodedData[i],HEX);
  }
   Serial.println(" ");
  delay(2000);
}
