import serial
import time

ardSerial = serial.Serial('/dev/ttyACM0', 115200)

while True:
	if (ardSerial.inWaiting()>0):
		#bytePacket = [ord(i) for i in serRead.read(3)]
		inData = [ord(i) for i in ardSerial.read(3)];
		#inData[0] = int(inData[0])
		#inData[1] = int(inData[1])
		#inData[2] = int(inData[2])
		print(inData)
		#print (inData[0], inData[1], inData[2])
		