
# LOCAL LIBRARIES
import config
import operationFuncs
import fallingStates
import torqueList


if __name__ == "__main__":

	ex2 = lambda: operationFuncs.ser.close() & exit()
	readArdStream = operationFuncs.readArdStream()
	readArdStream.start()
	#operationFuncs.serWrite.readline()


	# MENU 1
	menuOptions = { 
		1:('Calibrate', operationFuncs.calibrate),
		2:('Load Data Set', operationFuncs.genTorqueList),
		3:('Exit', ex2),
		}

	while not (config.calibrated and config.torqueListGenerated):
		operationFuncs.menuAndCalling(menuOptions)


	# MENU 2
	menuOptions = { 
		1:('Recalibrate', operationFuncs.calibrate),
		2:('Load New Data Set', operationFuncs.genTorqueList),
		3:('Initiate Fall Procedure', fallingStates.fallingSM),
		4:('Exit', ex2)
		}

	print (config.torqueManager.timeLimit)

	while True:
		operationFuncs.menuAndCalling(menuOptions)
