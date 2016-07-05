# LOCAL LIBRARIES
import config
import torqueList

# PYTHON LIBRARIES
import threading
import time
import os
import numpy as np
import struct

# ELECTRICAL LIBRARIES
#import roboclaw as rc
#import RPi.GPIO as GPIO
#from Arduino import Arduino 
import serial



######################### Setup Electrical Interfacing #########################

#motorDetected = False
#try:
#	motorDetected = rc.Open(config.roboclawPort, config.baudRate)
#except:
#	pass#

#if not motorDetected:
#	print('Motor port could not be opened.')
#	print('Port checked:')
#	print('\t'+config.roboclawPort)
#	print('Exiting...')
#	exit()


# SERIAL SETUP
ser = serial.Serial(config.ardPort, config.baudRate)
ser1 = serial.Serial(config.motorPort, config.baudRate)


# UPDATED FOR SERIAL
def setMotors(pwm_Knee, pwm_Hip):
	pwm_Knee, pwm_Hip = int(pwm_Knee), int(pwm_Hip)
	#pwm_Hip |= 0x80 #NEEDED FOR MOTOR 1
	#print ('pwm_knee = ', pwm_Knee, '\tpwm_Hip = ', pwm_Hip)
	dataStream = str(pwm_Knee) + ',' + str(pwm_Hip) + '\n'
	ser1.write(bytes(dataStream, 'utf-8'))
	

def readPot():               
	return config.potVal

def readAngles():
	hipAngle, kneeAngle = config.motorPos #hip is on encoder 2, knee is on encoder 1
	return kneeAngle, hipAngle, readPot()






######################### Electrical Interfacing Functions #########################


## IN PROGRESS
#def readCurrents():
##	kneeCurrent, hipCurrent = [0.01]*2
##	return np.array([kneeCurrent, hipCurrent]).reshape(2,1)/100.0
#	currents = rc.ReadCurrents(config.address)
#	if currents[0]:
#		kneeCurrent, hipCurrent = currents[1:] #make sure order is correct
#		return np.array([kneeCurrent, hipCurrent]).reshape(2,1)/100.0
#	else:
#		return -10*np.ones((2,1))



# COMPLETE
def calibrate():
	time.sleep(0.5)
	config.calibratedValues = readAngles()
	config.calibrated = True
	print('\tReference Positions: {}\n'.format(config.calibratedValues))
	return

# COMPLETE
def killMotors():
	##ser1.write(bytes((0,)))
	print (bytes(0,))
	return




######################### Threading Classes #########################

# COMPLETE
class fallConditionCheck (threading.Thread):
	def __init__(self, killEvent, successEvent):
		threading.Thread.__init__(self)
		self.killEvent = killEvent
		self.successEvent = successEvent
		self.angDiff_thresh = 1
		self.timeInterval = 0.05


	# TODO: modify run() to determine fall condition
	# Fall condition: read potentiometer, check if angular velocity above threshold, then run
	def run(self):
		while not self.killEvent.is_set():
			angle1 = readPot() # read potentiometer
			time.sleep(self.timeInterval)
			angle2 = readPot() # read potentiometer
			angDiff = (angle2-angle1)
			#print(angDiff)
			if angDiff > self.angDiff_thresh:
				self.successEvent.set()
				return





######################### Purely Software Functions #########################

# COMPLETE
def genTorqueList():
	config.torqueManager = torqueList.torqueListManager(config.torqueListPath)
	config.torqueListGenerated = True
	return


# COMPLETE
def menuAndCalling(menuOptions):

	numOptions = len(menuOptions)

	# Print option menu
	print('\nOptions:')
	for i in range(numOptions):
		print('{}. {}'.format(i+1, menuOptions[i+1][0]))
	choice = input('Choice: ')
	print('')

	try:
		choice = int(choice)
		assert choice in range(1,numOptions+1)
	except:
		print('Invalid choice. Choose again.')
		return

	menuOptions[choice][1]()

	rows, columns = os.popen('stty size', 'r').read().split()
	print('*'*int(columns))

	return





######################### TO BE CATEGORIZED #########################

# COMPLETE
def getPulseFromAngle(angleKnee, angleHip):
	position_Knee = angleKnee * config.pulsePerRotation//360.0 + config.calibratedValues[0]# + config.pulsePerRotation/4.0
	position_Hip  = angleHip  * config.pulsePerRotation//360.0 + config.calibratedValues[1]# + config.pulsePerRotation/4.0
	return position_Knee, position_Hip




# COMPLETE
class positionControl (threading.Thread):
	def __init__(self, killEvent):
		threading.Thread.__init__(self)
		self.killEvent = killEvent

	def run(self):
		target_KNEE, target_HIP = config.initialAngle_Knee, config.initialAngle_Hip
		dt = 0.02

		diff_KNEE, diff_HIP = 1e-9, 1e-9
		intError_KNEE, intError_HIP = 0, 0

		kP = 1.1
		kD = 0.0
		kI = 0.05

		while not self.killEvent.is_set():
			currentEncoder_KNEE = config.motorPos[1]
			diff_KNEE, oldDiff_KNEE = (target_KNEE - currentEncoder_KNEE), diff_KNEE
			propError_KNEE, dervError_KNEE, intError_KNEE, = diff_KNEE, (diff_KNEE - oldDiff_KNEE)/dt, intError_KNEE + diff_KNEE*dt
			speed_KNEE = int(max(min(kP*propError_KNEE + kI*dervError_KNEE + kD*intError_KNEE, 63), -63))
			speed_KNEE = -speed_KNEE + 64
			if abs(diff_KNEE) < 5:
				speed_KNEE = 64

				# railing negative is going to 0

			#print ('KNEE: ', diff_KNEE, [target_KNEE, currentEncoder_KNEE], speed_KNEE)

			currentEncoder_HIP = config.motorPos[0]
			diff_HIP, oldDiff_HIP = (target_HIP - currentEncoder_HIP), diff_HIP
			propError_HIP, dervError_HIP, intError_HIP, = diff_HIP, (diff_HIP - oldDiff_HIP)/dt, intError_HIP + diff_HIP*dt
			speed_HIP = int(max(min(kP*propError_HIP + kI*dervError_HIP + kD*intError_HIP, 63), -63))
			speed_HIP = speed_HIP + 191
			if abs(diff_HIP) < 5:
				speed_HIP = 191

			#print ('HIP: ', diff_HIP, [target_HIP, currentEncoder_HIP], speed_HIP|0x80)

			#print('')
			print ('target_knee = ', target_KNEE, '\tenc_knee = ', currentEncoder_KNEE, '\terr_knee = ', round(diff_KNEE, 2), 'pwm_knee = ', speed_KNEE, \
				'\t\ttarget_hip = ', target_HIP, '\tenc_hip = ', currentEncoder_HIP, '\terr_hip = ', round(diff_HIP, 2), 'pwm_hip = ', speed_HIP)

			#print ('err_knee = ', round(diff_KNEE, 2), 'pwm_knee = ', speed_KNEE, '\t\terr_hip = ', round(diff_HIP, 2), 'pwm_hip = ', speed_HIP)
			setMotors(speed_KNEE, speed_HIP)
			time.sleep(dt)

		return


# COMPLETE
# rising edge logic
def decode(byteIn, lastStates, motorList):
    bits = [(byteIn>>i)&1 for i in range(4)][::-1]
    if bits[0] and not lastStates[0]:
        motorList[0] += 1 if bits[1] else -1
    if bits[2] and not lastStates[1]:
        motorList[1] += 1 if not bits[3] else -1
    return motorList, [bits[0],bits[2]]

# COMPLETE
class readArdStream (threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.lastStates = [0]*2

	##def run(self):
	##	while True:
	##		if (ser.inWaiting()>0):
	##			bytePacket = ser.read(3)
	##			#bytePacket = ser.readline()
	##
	##			try:
	##				start = bytePacket.index(config.startByte)
	##			except:
	##				start = 0
	##
	##			bytePacket = bytePacket[start:]+bytePacket[:start]
	##
	##			config.potVal = bytePacket[1]
	##			config.motorPos, self.lastStates = decode(bytePacket[2], self.lastStates, config.motorPos)
	##			#print (config.potVal, config.motorPos)

	def run(self):
		while True:
			if (ser.inWaiting()>0):
				inData = ser.readline();
				inData = inData.decode("utf-8")
				rec = inData.split(',')
				strlen = len(rec)
				
				if (strlen == 4):
					for i in range(3):
						rec[i] = float(rec[i])
					#print (rec[0], rec[1], rec[2])
					config.potVal = rec[0]
					config.motorPos[1] = rec[1]
					config.motorPos[0] = rec[2]




				
if __name__ == "__main__":
	readArdStream = readArdStream()
	readArdStream.start()
	for i in range(1000):
		print('Knee: {2}\tHip: {1}\tHeel: {0}'.format(config.potVal, *config.motorPos))
		time.sleep(0.1)

#	exit()
