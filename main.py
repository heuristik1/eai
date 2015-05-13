#!/usr/bin/python
#import modules as MOD
from picreader import PICReader
import globals as GLB
import time

#only run if it is main
if __name__ == "__main__":

   #for debugging
   """
   MOD.createDebugFilePath()
   MOD.createLogFilePath()
   MOD.initSPI()
   MOD.setupPIC24RTC()
   MOD.collectTimeStampData()
   MOD.closeSPI()
   """
   preader = PICReader()
   preader.start()
   time.sleep(120)
   preader.join()
   

   #will need to wait for interrupt here
   #this should be a blocking call. When
   #interrupt comes in, trigger calibration code
   #once calibration is finished, call
   #preader.join(). For testing you could put a sleep in
   #i.e 
   """
   preader.start()
   sleep(120)
   preader.join()
   """
	
