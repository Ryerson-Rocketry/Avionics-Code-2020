import serial


inSerialBuffer = None


#  create global object for input buffer
def setPort(port, baudrate):
	global inSerialBuffer
	inSerialBuffer = serial.Serial(port, baudrate) 


#  get a valid 8 byte packet of data
def getPackets():
	while True:
		packet = list()                               #  create empty list to store bytes
		packet.append(inSerialBuffer.read())          #  read one byte from buffer
		header = (packet[0] & 0xe0) >> 5              #  store the header on its own
		
		goodToGo = True

		for i in range(7):                            #  read remaining 7 bytes in the packet
			packet.append(inSerialBuffer.read())    
			if ((packet[i+1] & 0xe0) >> 5) != header: #  if got a different header break and give signal to restart
				goodToGo = False
				break
		
		if goodToGo:                                  #  if everything is fine return the packet
			return packet


#  generate the checksum of a piece of data
def generateChecksum(data):
	result = data & 0x01                 #  LSB of lowest byte
	result |= (data & 0x80) >> 6         #  MSB of lowest byte
	result |= (data & 0x8000) >> 13      #  MSB of middle byte
	result |= (data & 0x10000) >> 13     #  LSB of highes byte

	return result


#  fix data to proper sign and decimal points
def fixData(data, header):
	if header == 0:                      #  if the header is 0 then it is barometer, has one decimal point, and is alsways positive
		data /= 10
		return data

	if (data & 0x800000) >> 23 == 1:     #  if the last bit is one then the number is negative
		data ^= 0x800000                 #  remove the last bit to get the actual value
		data *= -1                       #  multiply data by -1 to
	
	data /=10000

	return data
	

#  decode the packets to get data timestamp checksum and header
def decodePackets(packets):
	header = (packets[0] & 0xe0) >> 5    #  extract header from first byte   (0xe0 = 1110 0000)
	checksum = (packets[0] & 0x1e) >> 1  #  extract checlsum from first byte (0x1e = 0001 1110)

	data = (packets[0] & 0x01) << 23     #  extract data (0x01 = 0000 0001) 1000 0000 0000 0000 0000 0000
	data |= (packets[1] & 0x1f) << 18    #  extract data (0x1f = 0001 1111) 0111 1100 0000 0000 0000 0000
	data |= (packets[2] & 0x1f) << 13    #  extract data (0x1f = 0001 1111) 0000 0011 1110 0000 0000 0000
	data |= (packets[3] & 0x1f) << 8     #  extract data (0x1f = 0001 1111) 0000 0000 0001 1111 0000 0000
	data |= (packets[4] & 0x1f) << 3     #  extract data (0x1f = 0001 1111) 0000 0000 0000 0000 1111 1000
	data |= (packets[5] & 0x1c) >> 2     #  extract data (0x1f = 0001 1100) 0000 0000 0000 0000 0000 0111

	corrupted = False
	if checksum != generateChecksum(data):
		corrupted = True                 #  check for data corruption by checking the checksum

	data = fixData(data, header)         #  fix data

	time = (packets[5] & 0x03) << 10     #  extract time (0x03 = 0000 0011) 1100 0000 0000
	time |= (packets[6] & 0x1f) << 5     #  extract time (0x1f = 0001 1111) 0011 1110 0000
	time |= (packets[7] & 0x1f)          #  extract time (0x1f = 0001 1111) 0000 0001 1111

	return header, checksum, data, time, corrupted