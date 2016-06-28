
# PYTHON LIBRARIES
import datetime
import os

class ioOperations:

	# set as constant; 1 for Jan3_16-Jan9_16, start
	projectWeekStart = 1

	def __init__(self, dataDirectory):
		self.dataDirectory = dataDirectory
		self.getSessionDir()

	def getSessionDir(self):

		"""
		getSessionDir: Returns the directory for a given session to be used for saving new files. Previous session IDs are checked and a unique ID is created.

			Arguments:
			sourcePath -- 
		"""



		today = datetime.date.today()

		weekNum = today.isocalendar()[1]
		projectWeek = weekNum - ioOperations.projectWeekStart

		weekDay = today.weekday()
		weekStart = (today - datetime.timedelta(weekDay)).strftime("%b%d")
		weekEnd = (today + datetime.timedelta(6-weekDay)).strftime("%b%d")

		self.weekDir = 'week{0:02d}_{1:s}-{2:s}'.format(projectWeek, weekStart, weekEnd)
		weekDirFullPath = '{}/{}'.format(self.dataDirectory, self.weekDir)

		if not os.path.isdir( weekDirFullPath ):
			os.mkdir( weekDirFullPath )


		weekDayString = datetime.datetime.now().strftime("%A_%b-%d")
		self.sessionDir = '{0:s}/{1:s}'.format(weekDirFullPath, weekDayString)

		if not os.path.isdir( self.sessionDir ):
			os.mkdir( self.sessionDir )

		return



	def getSaveName(self):

		"""
		Returns the name of the next valid file. Previous files are checked and a unique ID is created.

			Arguments:
			path -- directory for the current session, found from getSessionDir()
		"""

		fileNums = [int(f.split('_')[0]) for f in os.listdir(self.sessionDir) if f.endswith('.csv')]

		if len(fileNums) == 0:
			fileNum = 1		
		else:
			fileNum = max(fileNums) + 1	

		timestamp = datetime.datetime.now().strftime("%a_%H_%M_%S")
		fileName = '{0:02d}_{1:s}.csv'.format(fileNum, timestamp)
		fullPath = '{0:s}/{1:s}'.format(self.sessionDir, fileName)

		return fileName, fullPath
