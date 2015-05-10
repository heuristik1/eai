#!/usr/bin/python
import modules as MOD

#only run if it is main
if __name__ == "__main__":

   #for debugging
   MOD.createDebugFilePath()

   MOD.createLogFilePath()
   MOD.initSPI()
   MOD.setupPIC24RTC()
   MOD.collectTimeStampData()
   MOD.closeSPI()	
