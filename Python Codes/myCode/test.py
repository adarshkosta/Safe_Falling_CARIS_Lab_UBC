import serial
import time

ardSerial = serial.Serial('/dev/ttyACM0', 38400)

data1 = str(40)
data2 = str(95)
data3 = "yayy"

data = data1 + ',' + data2 + ',' + data3 + '\n'
while 1 == 1:
	if (ardSerial.inWaiting()>0):
		inData = ardSerial.readline();
		inData = inData.decode("utf-8")
		rec = inData.split(',')
		strlen = len(rec)
		
		if (strlen == 4):
			for i in range(3):
				rec[i] = float(rec[i])
			print (rec[0], rec[1], rec[2])
		
		#print(inData)
	#ardSerial.write(bytes(data, 'utf-8'))
	