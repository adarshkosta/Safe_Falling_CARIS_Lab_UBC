
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

		time, heel_Torque, knee_Torque, hip_Torque = np.genfromtxt(self.torqueProfilePath, delimiter=',', unpack=True)

		self.fallTime = time[-1]
		pointsPerSecond = 1000

		self.resolution = 1.0 / pointsPerSecond
		self.numberOfTargets = int(self.fallTime*pointsPerSecond)

		kneeTorque_interpolator = interpolate.interp1d(time, knee_Torque, kind='cubic')
		hipTorque_interpolator = interpolate.interp1d(time, hip_Torque, kind='cubic')

		self.time_EXPANDED = np.arange(time[0], self.fallTime-self.resolution, self.resolution)
		self.kneeTorque_EXPANDED = kneeTorque_interpolator(self.time_EXPANDED)
		self.hipTorque_EXPANDED = hipTorque_interpolator(self.time_EXPANDED)

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

		torques = np.array([[self.kneeTorque_EXPANDED[index]], [self.hipTorque_EXPANDED[index]]])

		return torques

