
# LOCAL LIBRARIES
import operationFuncs
import dataSaving
import config

# PYTHON LIBRARIES
import time
import threading
import numpy as np
import collections


class fallingSM:

	dataSizeTuple = (20000, 6)

	def __init__(self):

		self.fallingData = np.zeros(fallingSM.dataSizeTuple)
		self.initiateFallMode()


	# STATE MACHINE IMPLEMENTATION (COMPLETE)
	def initiateFallMode(self):

		"""
		Defines and implements falling state machine and controls program flow.

			Arguments:
				(None)

			Returns:
				(None)

		"""

		while True:
			# give option to go to neutral, primed, or quit
			nextStep = self.push_into_position_func()
			if nextStep == 'N': # stay in neutral state
				varNeutral = self.neutral_state()
				if varNeutral == 'R':
					continue
				elif varNeutral == 'M':
					return
			elif nextStep == 'M': # quit back to Main Menu
				return

			self.primed_state()
			self.falling()
			self.save_data()
			return


	# STATE 1 (COMPLETE)
	def push_into_position_func(self):

		"""
		Applies constant torques to push apparatus into a specified position.
		Reads config file to get target positions relative to calibrated values.

			Arguments:
				(None)

			Returns:
				nextState -- Desired next state based on user input; 'N' for Neutral, 'P' for Primed, 'M' for Main Menu.

		"""


		global killCondition_push
		killCondition_push = threading.Event()
		positionControl = operationFuncs.positionControl(killCondition_push)
		positionControl.start()
		print('\nSTARTING!')

		while True:
			var = input('Options: N (Neutral) / P (Primed) / M (Main Menu)\n')
			if var in ['N', 'P', 'M']:
				#killCondition_push.set()
				return var
			else:
				print('Invalid. Choose \'N\', \'P\', or \'M\'.\n')



	# WAIT STATE (COMPLETE)
	def neutral_state(self):

		"""
		Turns off motors and waits for user input to continue.

			Arguments:
				(None)

			Returns:
				nextState -- Desired next state based on user input; 'M' for Main Menu, 'R' to Restart control loop.

		"""
		killCondition_push.set()
		time.sleep(0.3)
		operationFuncs.killMotors()
		print('\nNEUTRAL: Apparatus is set to neutral and awaiting feedback to restart.')
		while True:
			var = input('Options: M (Main Menu) / R (Restart Falling)\n')
			if var in ['M', 'R']:
				return var
			else:
				print('Invalid. Choose \'M\', or \'R\'.\n')


	# STATE 2 (COMPLETE)
	def primed_state(self):

		"""
		Checks to see if falling condition is met, then transitions to falling.

			Arguments:
				(None)

			Returns:
				(None)

		"""

		print('\nPRIMED!')

		# threading isn't really at all important here; should probably change this
		killCondition = threading.Event()
		successCondition = threading.Event()

		fallChecker = operationFuncs.fallConditionCheck(killCondition, successCondition)
		fallChecker.start()

		while not successCondition.is_set():
			continue

		killCondition.set()

		return


	# STATE 3 (IN PROGRESS)
	def falling(self):

		"""
		Implements control scheme.

			Arguments:
				(None)

			Returns:
				Data -- Data structure to be saved

		"""

		print('\nFALLING!\n')

		killCondition_push.set()

		lastError_Knee, lastError_Hip, pwm_Knee, pwm_Hip, dataIndex = [0]*5

		kneeAngle, hipAngle, heelAngle = config.calibratedValues
		angVel_knee, angVel_hip, angVel_heel = [0]*3
		angAcc_knee, angAcc_hip, angAcc_heel = [0]*3

		currents = np.zeros((2,1))
		map12Volts = np.zeros((2,1))
		getByteChunk = np.zeros((2,1))

		intError = np.zeros((2,1))
		intErrorMax = 1000

		lastPWM, outputVoltages = np.array([0,0]).reshape(2,1), np.array([0,0]).reshape(2,1)

		timeStart, timeLast = time.time(), 0
		timeNow, timeStep = time.time() - timeStart, 10e-9 # initialize timeStep as small value

		while True:
			
			Ki = 1

			# READ PRECOMPUTED DATA FROM MEMORY
			desiredTorques = config.torqueManager.torqueGoals(timeNow)

			# READ DATA FROM SENSORS AND CALCULATE DERIVED DATA
			#last_kneeAngle, last_hipAngle, last_heelAngle = kneeAngle, hipAngle, heelAngle
			kneeAngle, hipAngle, heelAngle = operationFuncs.readAngles()

			#last_angVel_knee, last_angVel_hip, last_angVel_heel = angVel_knee, angVel_hip, angVel_heel
			#angVel_knee, angVel_hip, angVel_heel = np.array([kneeAngle-last_kneeAngle,
			#	hipAngle-last_hipAngle, heelAngle-last_heelAngle]) / timeStep

			lastThingyThing = 10

			#last_angVel_knee, last_angVel_hip, last_angVel_heel = angVel_knee, angVel_hip, angVel_heel
			angVel_knee, angVel_hip, angVel_heel = np.array([kneeAngle-self.fallingData[max(dataIndex-lastThingyThing-1,0), 1],
				hipAngle-self.fallingData[max(dataIndex-lastThingyThing-1,0), 2], heelAngle-self.fallingData[max(dataIndex-lastThingyThing-1,0), 3]]) / timeStep

			#last_angAcc_knee, last_angAcc_hip, last_angAcc_heel = angAcc_knee, angAcc_hip, angAcc_heel
			#angAcc_knee, angAcc_hip, angAcc_heel = np.array([angVel_knee-last_angVel_knee, 
			#	angVel_hip-last_angVel_hip, angVel_heel-last_angVel_heel]) / timeStep

			actualTorques = config.torque_motorConstant * np.sign(lastPWM) * currents

			# CONTROL OPERATIONS
			#EM_torqueFeedback_adjust = actualTorques
			EM_torque_adjust = desiredTorques # - torqueResponses
			errorNow = EM_torque_adjust - actualTorques


			# INTEGRAL ERROR AND CURRENT CALCULATION
			intError += errorNow*timeStep
			lastPWM = outputVoltages
			outputVoltages = Ki*intError; 
			currents = (outputVoltages - config.angVel_motorConstant*64*np.array([angVel_knee, angVel_hip]).reshape(2,1))\
						/config.armRes_motorConstant
						
			# GET PWM INPUT FROM VOLTAGES
			##map12Volts = lambda x: max(min(x, 6),-6)+6
			##getByteChunk = lambda x: 1+round(x * 126.0/12)
			##pwm_Knee, pwm_Hip = map(getByteChunk, map(map12Volts, outputVoltages))

			map12Volts[0] = max(min(outputVoltages[0], 6),-6)+6
			map12Volts[1] = max(min(outputVoltages[1], 6),-6)+6
			getByteChunk[0] = 1 + int(map12Volts[0] * 126.0/12)
			getByteChunk[1] = 1 + int(map12Volts[1] * 126.0/12)
			pwm_Knee, pwm_Hip = getByteChunk

			# CONTROL JOINTS
			##operationFuncs.setMotors(pwm_Knee=pwm_Knee, pwm_Hip=pwm_Hip)
			print(pwm_Knee, pwm_Hip)
			# ASSIGN DATA TO DATA ARRAY
			unpackList = lambda list2Unpack: [list2Unpack[0][0],list2Unpack[1][0]]
			self.fallingData[dataIndex, :] = [timeNow, kneeAngle, hipAngle, heelAngle] + unpackList(actualTorques.tolist())
			dataIndex += 1

			# UPDATE TIME CONDITIONS
			timeLast, timeNow = timeNow, time.time() - timeStart
			timeStep = timeNow - timeLast
			print(timeNow)
			if timeNow > config.torqueManager.timeLimit:
				break

		operationFuncs.killMotors()

		self.fallingData = self.fallingData[:dataIndex]

		return



	# STATE 4 (COMPLETE)
	def save_data(self):

		"""
		Takes data and saves in format specified by dataSaving module.

			Arguments:
				Data -- Data to be saved
				Directory -- Base directory to be used for saving (defined by settings, date)

			Returns:
				(None)

		"""

		print('\nSAVING!')

		ioStructure = dataSaving.ioOperations(config.dataFolder)
		fileName, fullPath = ioStructure.getSaveName()


		metadataList = ['{}\t{}'.format(name, eval('config.'+name)) for name in config.metadataNames]
		metadataForFile = '\n'.join(metadataList)
		metadataForPrinting = '\t' + '\n\t'.join(metadataList)
		print(metadataForPrinting)

		np.savetxt(fullPath, self.fallingData, delimiter=',', header=metadataForFile, comments='')

		print('\nData Saved!')
		print('\tSize {}'.format(self.fallingData.shape))
		print('\t{} lines of metadata'.format(len(config.metadataNames)))
		print('\tPath name: {}\n'.format(fileName))

		return



