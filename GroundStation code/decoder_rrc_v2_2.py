import serial

inSerialBuffer = None


def setPort(port, baudrate):
	'''
	setPort(port, baudrate) --> None
	
	Create global object for input buffer.

	params:
	port -- Port number, either "COM*" for windows or "tty*" for nix systems.
	baudrate -- Baud rate for connection.
	'''
	global inSerialBuffer
	inSerialBuffer = serial.Serial(port, baudrate) 


def getPackets():
	'''
	getPackets() --> list
	
	Get a valid 8 byte packet of data.

	Params:
	None

	Return:
	An 8 byte list of the same header type.
	'''
	goodToGo = True

	while True:
		if goodToGo:                                  #  read first byte from buffer and store it
			packet = [int.from_bytes(inSerialBuffer.read(), byteorder="big", signed=False)]
		
		header = (packet[0] & 0xe0) >> 5              #  store the header on its own
		
		goodToGo = True
		for i in range(6):                            #  read 6 more bytes
			packet.append(int.from_bytes(inSerialBuffer.read(), byteorder="big", signed=False)) #  read one byte from buffer
			if ((packet[i+1] & 0xe0) >> 5) != header: #  if got a different header break and give signal to restart
				goodToGo = False
				packet = [packet[-1]]                 #  set last byte as the first one to restart getting packets
				break
		
		if not goodToGo:                              #  restart if not good to go
			continue
		
		packet.append(int.from_bytes(inSerialBuffer.read(), byteorder="big", signed=False)) #  read the last byte
		if (packet[-1] & 0xe0) != 0xe0:               #  check last byte has ending header, if not then there was corruption
			goodToGo = False
			packet = [packet[-1]]                     #  otherwise store it as a packet beginning and restart

		if goodToGo:                                  #  return value if everything is fine
			return packet


def generateChecksum(data):
	'''
	generateChecksum(data) --> int

	Generate the checksum of a piece of data.

	Params:
	data -- A 3 byte integer.

	Return:
	An integer with the least 4 bits set to the checksusm
	'''
	result = data & 0x01                              #  LSB of lowest byte
	result |= (data & 0x80) >> 6                      #  MSB of lowest byte
	result |= (data & 0x8000) >> 13                   #  MSB of middle byte
	result |= (data & 0x10000) >> 13                  #  LSB of highes byte

	return result


def fixData(data, header):
	'''
	fixData(data, header) --> int

	Fix data to proper sign and decimal points.

	Params:
	data -- Raw integer recieved.
	header -- the header, aka type of data received.

	Return:
	The data with the proper sign and decimal points
	'''
	if header == 0:                                   #  if the header is 0 then it is barometer, has one decimal point, and is alsways positive
		data /= 100
		return data

	if (data & 0x800000) >> 23 == 1:                  #  if the last bit is one then the number is negative
		data ^= 0x800000                              #  remove the last bit to get the actual value
		data *= -1                                    #  multiply data by -1 to
	
	data /=10000

	return data
	

def decodePackets(packets):
	'''
	decodePackets(packets) --> tuple
	
	Decode the packets to get data timestamp checksum and header.

	Params:
	packets -- The 8 byte list recieved using the getPackets function

	Return:
	A tuple with the following variables.
	[0] -- int(header) -- The header in the packets.
	[1] -- int(checksum) -- The checksum sent in the packets for later verification.
	[2] -- float(data) -- Fixed data to proper decimal point and sign.
	[3] -- int(time) -- The time stamp of the data when sent.
	[4] -- bool(corrupted) -- True if the calculated checksum doesn't match the recieved checksum False otherwise.
	'''
	header = (packets[0] & 0xe0) >> 5                 #  extract header from first byte   (0xe0 = 1110 0000)
	checksum = (packets[0] & 0x1e) >> 1               #  extract checlsum from first byte (0x1e = 0001 1110)

	data  = (packets[0] & 0x01) << 23                 #  extract data (0x01 = 0000 0001) 1000 0000 0000 0000 0000 0000
	data |= (packets[1] & 0x1f) << 18                 #  extract data (0x1f = 0001 1111) 0111 1100 0000 0000 0000 0000
	data |= (packets[2] & 0x1f) << 13                 #  extract data (0x1f = 0001 1111) 0000 0011 1110 0000 0000 0000
	data |= (packets[3] & 0x1f) << 8                  #  extract data (0x1f = 0001 1111) 0000 0000 0001 1111 0000 0000
	data |= (packets[4] & 0x1f) << 3                  #  extract data (0x1f = 0001 1111) 0000 0000 0000 0000 1111 1000
	data |= (packets[5] & 0x1c) >> 2                  #  extract data (0x1f = 0001 1100) 0000 0000 0000 0000 0000 0111

	corrupted = False
	if checksum != generateChecksum(data):
		corrupted = True                              #  check for data corruption by checking the checksum
	data = fixData(data, header)                      #  fix data

	time  = (packets[5] & 0x03) << 10                 #  extract time (0x03 = 0000 0011) 1100 0000 0000
	time |= (packets[6] & 0x1f) << 5                  #  extract time (0x1f = 0001 1111) 0011 1110 0000
	time |= (packets[7] & 0x1f)                       #  extract time (0x1f = 0001 1111) 0000 0001 1111

	return header, checksum, data, time, corrupted