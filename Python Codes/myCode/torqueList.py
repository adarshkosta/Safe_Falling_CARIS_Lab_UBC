
# LOCAL LIBRARIES
import config

# PYTHON LIBRARIES
import numpy as np
from scipy import interpolate


class torqueListManager:

	def __init__(self, torqueProfilePath):
		self.torqueProfilePath = torqueProfilePath
		self.genTorqueList()

	def genTorqueList(self):

		time, knee_Torque, hip_Torque, knee_Response, hip_Response, Control_1, Control_2, Control_3, Control_4 = \
			np.genfromtxt(self.torqueProfilePath, delimiter=',', unpack=True)

		self.fallTime = time[-1]
		pointsPerSecond = 10000

		self.resolution = 1.0 / pointsPerSecond
		self.numberOfTargets = int(self.fallTime*pointsPerSecond)

		kneeTorque_interpolator = interpolate.interp1d(time, knee_Torque, kind='cubic')
		hipTorque_interpolator = interpolate.interp1d(time, hip_Torque, kind='cubic')
		kneeResponse_interpolator = interpolate.interp1d(time, knee_Response, kind='cubic')
		hipResponse_interpolator = interpolate.interp1d(time, hip_Response, kind='cubic')
		Control1_interpolator = interpolate.interp1d(time, Control_1, kind='cubic')
		Control2_interpolator = interpolate.interp1d(time, Control_2, kind='cubic')
		Control3_interpolator = interpolate.interp1d(time, Control_3, kind='cubic')
		Control4_interpolator = interpolate.interp1d(time, Control_4, kind='cubic')

		self.time_EXPANDED = np.arange(time[0], self.fallTime-self.resolution, self.resolution)
		self.kneeTorque_EXPANDED = kneeTorque_interpolator(self.time_EXPANDED)
		self.hipTorque_EXPANDED = hipTorque_interpolator(self.time_EXPANDED)
		self.kneeResponse_EXPANDED = kneeResponse_interpolator(self.time_EXPANDED)
		self.hipResponse_EXPANDED = hipResponse_interpolator(self.time_EXPANDED)
		self.Control1_EXPANDED = Control1_interpolator(self.time_EXPANDED)
		self.Control2_EXPANDED = Control2_interpolator(self.time_EXPANDED)
		self.Control3_EXPANDED = Control3_interpolator(self.time_EXPANDED)
		self.Control4_EXPANDED = Control4_interpolator(self.time_EXPANDED)

		self.timeLimit = self.time_EXPANDED[-1]

		print('\tTorque Profile: {}'.format(config.torqueListPath))
		print('\tResolution (s): {}'.format(self.resolution))
		print('\tPoints Per Second: {}'.format(pointsPerSecond))
		print('\tFalltime (s): {}'.format(self.fallTime))
		print('\tResolution Multiplier: {}\n'.format(1.0*len(self.time_EXPANDED)/len(time)))


	def torqueGoals(self, timeVal):
		index = int(timeVal/self.resolution)
		if timeVal > self.timeLimit:
			return

		C1 = self.Control1_EXPANDED[index]
		C2 = self.Control2_EXPANDED[index]
		C3 = self.Control3_EXPANDED[index]
		C4 = self.Control4_EXPANDED[index]

		controlMat = np.array([[C1,C2],[C3,C4]])
		torques = np.array([[self.kneeTorque_EXPANDED[index]], [self.hipTorque_EXPANDED[index]]])
		torqueResponses = np.array([[self.kneeResponse_EXPANDED[index]], [self.hipResponse_EXPANDED[index]]])

		return torques, torqueResponses, controlMat

