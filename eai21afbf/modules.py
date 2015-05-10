#!/usr/bin/python
import time, spidev, os, sys, math
import globals as GLB 
import RPi.GPIO as GPIO ##Import GPIO library
import plotly.plotly as py # plotly library
import json # used to parse config.json
import datetime # log and plot current time

#plotly variables

with open('/home/pi/eai-proj010315/config.json') as config_file:
 	plotly_user_config = json.load(config_file)

 	py.sign_in(plotly_user_config["plotly_username"], plotly_user_config["plotly_api_key"]) 

stream = py.Stream(plotly_user_config['plotly_streaming_tokens'][0])
stream.open()

url = py.plot([
    {
        'x': [], 'y': [], 'type': 'scatter',
        'stream': {
            'token': plotly_user_config['plotly_streaming_tokens'][0],
            'maxpoints': 86400
        }
    }], filename='LotusRoot')
    
print "View your streaming graph here: ", url

#collection of modules for general usage

#for handling of script inputs
def getSPIOutputByte(input):
   msb = input >> 8;
   lsb = input & 0xff;
   GLB.spi.writebytes([msb, lsb])
   data = GLB.spi.readbytes(2)
   return data

def get16bitSPIData(returnData):
   return getWord(returnData[0], returnData[1])      

def getWord(msb, lsb):
   return (msb*256+lsb)

#convert decimal to hex value
def decToHex(decValue):
   firstDigit = (decValue / 10) << 4
   secondDigit = decValue % 10
   return (firstDigit + secondDigit)

def getSystemTimeInHex():
   rawTimeData = (time.strftime("%y/%m/%d/%H/%M/%S")).split("/")
   for i in range (0, 6):
      rawTimeData[i] = decToHex(int(rawTimeData[i]))
   return rawTimeData

def addSPIDataMarking(sendData):
   return (sendData | 0x8000)

def removeSPIDataMarking(receiveData):
   return (receiveData & 0x7fff)

def sendSPIDataWithMarking(SPIOutput):
   return getSPIOutputByte(addSPIDataMarking(SPIOutput))

def createCommandData(commandType, rawData):
   return ((commandType<<8) + rawData)

#get approximate values of temperature reading based on equation
def getTempReading(voltVal):
   fahrenheitVal = (-4.71728 * voltVal**7) + (57.51 * voltVal**6) + \
      (-290.91 * voltVal**5) + (789.2 * voltVal**4) + \
      (-1247.4 * voltVal**3) + (1176.45 * voltVal**2) + \
      (-704.13 * voltVal) + 343.992
   celciusVal = fahrenheitToCelcius(fahrenheitVal)
   return celciusVal

#convert fahrenheit to celcius
def fahrenheitToCelcius(fahrenheightVal):
   return ((fahrenheightVal-32.0) * (5.0/9.0))

#get current value
def getCurrReading(voltVal):
   currentVal = (voltVal - GLB.vref) * 125
   if (voltVal<GLB.vref):
      currentVal = (GLB.vref - voltVal) * -125
   return currentVal

#get last 6 digit of mac and use it as an id
def getDeviceId():
    # Read MAC from file
    myMAC = open('/sys/class/net/eth0/address').read()
    macToken = myMAC.split(":")
    uniqueId = macToken[3] + macToken[4] + macToken[5] 
    return uniqueId.replace("\n", "")

#use this function to reset pic24
def resetPic24():
   initPicResetGPIO()
   GPIO.output(GLB.NRESET_PIC24_GPIO_PIN,False)
   time.sleep(GLB.NRESET_PIC24_HOLD_TIME)
   GPIO.output(GLB.NRESET_PIC24_GPIO_PIN,True)
   releaseGPIO()
   return

#use this function to reset entire system
def resetSystem():
   initNPorSysGPIO()
   GPIO.output(GLB.NPOR_SYS_GPIO_PIN,False)
   time.sleep(GLB.NPOR_SYS_HOLD_TIME)
   GPIO.output(GLB.NPOR_SYS_GPIO_PIN,True)
   releaseGPIO()
   return

#open connection and setup spi so we can talk to the pic
def initSPI():
   #GLB.spi = spidev.SpiDev()
   GLB.spi.open(0,0)
   return

#close spi connection
def closeSPI():
   GLB.spi.close()
   return

#init pic reset gpio
def initPicResetGPIO():
   GPIO.setwarnings(False)
   GPIO.setmode(GPIO.BCM) ## Use bcm numbering
   GPIO.setup(GLB.NRESET_PIC24_GPIO_PIN, GPIO.OUT) ## Setup NRESET_PIC24_PIN as output
   return

#init npor gpio
def initNPorSysGPIO():
   GPIO.setwarnings(False)
   GPIO.setmode(GPIO.BCM) ## Use board pin numbering
   GPIO.setup(GLB.NPOR_SYS_GPIO_PIN, GPIO.OUT) ## Setup NPOR_SYS_PIN as output
   return

#release gpio pins
def releaseGPIO():
   GPIO.cleanup()
   return

#find vref
def findVref():
   myMacAddress = getDeviceId()
   if (myMacAddress==GLB.DEVICE_ID_816874):
      GLB.vref = GLB.MAC_ADDRESS_816874_VREF
   elif (myMacAddress==GLB.DEVICE_ID_8daf22):
      GLB.vref = GLB.MAC_ADDRESS_8daf22_VREF
   elif (myMacAddress==GLB.DEVICE_ID_8ece5a):
      GLB.vref = GLB.MAC_ADDRESS_8ece5a_VREF
   else:
      GLB.vref = GLB.MAC_ADDRESS_OTHER_VREF
   return

#set up rtc time in pic
def setupPIC24RTC():

   #only init rtc if we never done it before
   if (not(os.path.isdir(GLB.RTC_FILE_PATH))):
   #if (True):

      #init retry attempt counter
      retryAttempt = 0

      currentPICResetCount = 0

      #get sys time in hex
      sysTimeHex = getSystemTimeInHex()
      rtcSetupDone = False

      while (not rtcSetupDone):
         for i in range (GLB.SET_RTC_YEAR, GLB.SET_RTC_SECOND+1):
            my16bitSPIData = get16bitSPIData(sendSPIDataWithMarking(createCommandData(i, sysTimeHex[i])))
            SPIAck = removeSPIDataMarking(my16bitSPIData)

            #no ack from pic that we set up rtc
            if (not SPIAck):
               #try reset spi module on pi to fix problem
               closeSPI()
               initSPI()
               retryAttempt = retryAttempt + 1
               log(GLB.DEBUG_SAVE, "rtc variable " + str(i) + " set up failed, retry setup in .1 second, attemp #" + str(retryAttempt) + "\n")
               log(GLB.debugType, "rtc variable " + str(i) + " set up failed, retry setup in .1 second, attemp #" + str(retryAttempt) + "\n")
               time.sleep(GLB.SETUP_FAILED_DELAY)

               #reset pic if max number of retry attempts is reached
	       if (retryAttempt>=GLB.MAX_RETRY_ATTEMPT):

                  #reset system if max number of resets on pic still does not resolve issue 
	          if (currentPICResetCount>=GLB.MAX_PIC_RESET):
                     log(GLB.debugType, "max number of pic reset for rtc setup reached, resetting system\n")
                     #catastrophic failure, reset entire board and exit program
                     resetSystem()
                     sys.exit()

                  #reset pic to see if we can fix the rtc setup issue
                  else:
                     log(GLB.debugType, "max number of retry attempt for rtc setup is reached, resetting pic\n")
                     resetPic24()
   	             time.sleep(GLB.NRESET_PIC24_HOLD_TIME*2)
                     retryAttempt = 0
                     currentPICResetCount = currentPICResetCount + 1
               break

            #rtc variable got set up correctly 
            log(GLB.debugType, "rtc variable " + str(i) + " set up correctly\n")
         
            #rtc setup is done
            if (i==GLB.SET_RTC_SECOND):
               log(GLB.debugType, "rtc set up successful\n")
               rtcSetupDone = True
               createRTCFilePath() #inidicate we setup rtc so we don't need to do it in the future
               time.sleep(GLB.PIC_SETUP_DELAY)
   return

#collect time stamp data
def collectTimeStampData():

   #find vref
   findVref()
   log(GLB.debugType, "vref = " + str(GLB.vref) + "\n")

   #init retry attempt counter
   retryAttempt = 0

   dataCount = 0
   currentFileName = GLB.LOG_FILE_NAME
   currentPICResetCount = 0
   lastSecond = -1

   #variable used for time sync with pic calibration
   selfCalibratedWaitTime = GLB.TS_COLLECTION_DELAY

   #for debugging purposes
   curentDebugFileName = getDebugNewFileName()

   #scan pic every 1 sec for new adc data
   while (True):

      #prepare time stamp in pic
      timeStampDone = False

      while (not timeStampDone):
         my16bitSPIData = get16bitSPIData(sendSPIDataWithMarking(createCommandData(GLB.PREPARE_TS, GLB.NULL)))
         SPIAck = removeSPIDataMarking(my16bitSPIData)
         
         #no ack from pic that we set up time stamp
         if (not SPIAck):
            #try reset spi module on pi to fix problem
            closeSPI()
            initSPI()
            retryAttempt = retryAttempt + 1
            log(GLB.DEBUG_SAVE, "prepare time stamp failed, retry setup in .1 second, attemp #" + str(retryAttempt) + "\n")
            log(GLB.debugType, "prepare time stamp failed, retry setup in .1 second, attemp #" + str(retryAttempt) + "\n")
            time.sleep(GLB.SETUP_FAILED_DELAY)

            #reset pic if max number of retry attempts is reached
	    if (retryAttempt>=GLB.MAX_RETRY_ATTEMPT):
               
               #reset system if max number of resets on pic still does not resolve issue 
	       if (currentPICResetCount>=GLB.MAX_PIC_RESET):
                   log(GLB.debugType, "max number of pic reset  for ts collection reached, resetting system\n")
                   #catastrophic failure, reset entire board and exit program
                   resetSystem()
                   sys.exit()
               
               #reset pic to see if we can fix the ts collection issue
               else:
                  log(GLB.debugType, "max number of retry attempt for ts collection is reached, resetting pic\n")
                  resetPic24()
	          time.sleep(GLB.NRESET_PIC24_HOLD_TIME*2) 
	          retryAttempt = 0
                  currentPICResetCount = currentPICResetCount + 1
           
         else:
            log(GLB.debugType, "prepare time stamp success\n")
            timeStampDone = True
            time.sleep(GLB.PIC_SETUP_DELAY)
	    retryAttempt = 0

      #get time stamp data in pic
      TSdata = []

      #get a new file name in the beginning of the data collection cycle
      if (dataCount == 0):  
         time_1 = time.time()
	 currentFileName = getNewFileName()
         #for debugging
         curentDebugFileName = getDebugNewFileName()

      for i in range (GLB.GET_RTC_YEAR, GLB.GET_ADC_DATA6+1):
         my16bitSPIData = get16bitSPIData(sendSPIDataWithMarking(createCommandData(i, GLB.NULL)))
         SPIData = removeSPIDataMarking(my16bitSPIData)
	 TSdata.append(SPIData)
         
      #convert adc raw data to meaningful data
      picTime = str(TSdata[GLB.TS_DATA_MONTH]) + "/" + \
         str(TSdata[GLB.TS_DATA_DAY]) + "/" + \
         str(TSdata[GLB.TS_DATA_YEAR]) + " " + \
         str(TSdata[GLB.TS_DATA_HOUR]) + ":" + \
         str(TSdata[GLB.TS_DATA_MINUTE]) + ":" + \
         str(TSdata[GLB.TS_DATA_SECOND])
      picTime = time.strftime("%Y-%m-%d %H:%M:%S")
      v1Reading = str(round(TSdata[GLB.TS_DATA_V1] * GLB.ADC_3_3V_RATIO * GLB.VOLTAGE_ADC_RATIO, GLB.DECIMAL_ACCURACY))
      v2Reading = str(round(TSdata[GLB.TS_DATA_V2] * GLB.ADC_3_3V_RATIO * GLB.VOLTAGE_ADC_RATIO, GLB.DECIMAL_ACCURACY))
      v3Reading = str(round(TSdata[GLB.TS_DATA_V3] * GLB.ADC_3_3V_RATIO * GLB.VOLTAGE_ADC_RATIO, GLB.DECIMAL_ACCURACY))
      t1Reading = str(round(getTempReading(TSdata[GLB.TS_DATA_T1] * GLB.ADC_3_3V_RATIO), GLB.DECIMAL_ACCURACY))
      t2Reading = str(round(getTempReading(TSdata[GLB.TS_DATA_T2] * GLB.ADC_3_3V_RATIO), GLB.DECIMAL_ACCURACY))
      c1Reading = str(round(getCurrReading(TSdata[GLB.TS_DATA_C1] * GLB.ADC_3_3V_RATIO), GLB.DECIMAL_ACCURACY))      
      c2Reading = str(round(getCurrReading(TSdata[GLB.TS_DATA_C2] * GLB.ADC_3_3V_RATIO), GLB.DECIMAL_ACCURACY))      

      collectedData = picTime + "," + getDeviceId() + "," + \
            v1Reading + "," + v2Reading + "," + v3Reading + "," + \
            c1Reading + "," + c2Reading + "," + \
            t1Reading + "," + t2Reading + "\n"

      #check if data pass range test
      validVoltage1Range = verfiyDataRange(GLB.DATA_TYPE_VOLTAGE, float(v1Reading))
      validVoltage2Range = verfiyDataRange(GLB.DATA_TYPE_VOLTAGE, float(v2Reading))
      validVoltage3Range = verfiyDataRange(GLB.DATA_TYPE_VOLTAGE, float(v3Reading))
      validCurrent1Range = verfiyDataRange(GLB.DATA_TYPE_CURRENT, float(c1Reading ))
      validCurrent2Range = verfiyDataRange(GLB.DATA_TYPE_CURRENT, float(c2Reading ))
      validTemperature1Range = verfiyDataRange(GLB.DATA_TYPE_TEMPERATURE, float(t1Reading))
      validTemperature2Range = verfiyDataRange(GLB.DATA_TYPE_TEMPERATURE, float(t2Reading))

      validAllRange = (validVoltage1Range and validVoltage2Range and validVoltage3Range and \
         validCurrent1Range and validCurrent2Range and \
         validTemperature1Range and validTemperature2Range)

      #only store values if all range pass test
      if (validAllRange):

         currentSecond = int(TSdata[GLB.TS_DATA_SECOND])
         identicalSecondFound = False

         #calibrate wait time if we have to
         if (lastSecond>=0):
            #got identical time, slow down waiting time
            if (currentSecond==lastSecond):
               selfCalibratedWaitTime = selfCalibratedWaitTime + GLB.MIN_CALIBRATE_DECREMENT
               log(GLB.debugType, "slowing down calibration time, new wait time = " + str(selfCalibratedWaitTime))
               identicalSecondFound = True
            else:
               if (currentSecond==0):
                  currentSecond = 60
               #got time gap differential not equal to one second, speed up waiting time
               if (currentSecond-lastSecond!=1):   
                  log(GLB.debugType, "time differential = " + str(currentSecond-lastSecond))
                  selfCalibratedWaitTime = selfCalibratedWaitTime - (GLB.MIN_CALIBRATE_DECREMENT*1000)
                  log(GLB.debugType, "speeding up calibration time, new wait time = " + str(selfCalibratedWaitTime))

                  if (selfCalibratedWaitTime<GLB.TS_COLLECTION_DELAY):
                     selfCalibratedWaitTime = GLB.TS_COLLECTION_DELAY
                     log(GLB.debugType, "restoring to default calibration time, new wait time = " + str(selfCalibratedWaitTime))

         lastSecond = (currentSecond%60)

         if (not identicalSecondFound): 
      
            dataCount  = dataCount + 1
     
            #store data  
            storeToFile(currentFileName, collectedData)
            
            #for debugging purposes
            #storeToFile(curentDebugFileName, collectedData) 

            #reset counter            
            if (dataCount>=GLB.TS_DURATION):
               dataCount = 0 
      
            log(GLB.debugType, "[TIME] " + picTime)
            log(GLB.debugType, "[ID] " + getDeviceId())
            log(GLB.debugType, "[V1] " + v1Reading)
            log(GLB.debugType, "[V2] " + v2Reading)
            log(GLB.debugType, "[V3] " + v3Reading)
            log(GLB.debugType, "[C1] " + c1Reading)
            log(GLB.debugType, "[C2] " + c2Reading)
            log(GLB.debugType, "[T1] " + t1Reading)
            log(GLB.debugType, "[T2] " + t2Reading + "\n")
            
            #Plot Data
            stream.write({'x': datetime.datetime.now(), 'y': c1Reading})
         #set delay for calibrated time
         time_2 = time.time()
	 if (time_2 - time_1) < GLB.TS_COLLECTION_DELAY:
		no_of_sleeps = int(round((GLB.TS_COLLECTION_DELAY - (time_2 - time_1)) / 0.1))
		for i in range(no_of_sleeps):
			time.sleep(0.1)

      else:

         #try reset spi module on pi to fix problem
         closeSPI()
         initSPI()
        
         errSensor = ""
         if (not validVoltage1Range):
	    errSensor = "voltage1 "
         if (not validVoltage2Range):
            errSensor = errSensor + "voltage2 "
         if (not validVoltage3Range):
            errSensor = errSensor + "voltage3 "
         if (not validCurrent1Range):
            errSensor = errSensor + "current1 "
         if (not validCurrent2Range):
            errSensor = errSensor + "current2 "
         if (not validTemperature1Range):
            errSensor = errSensor + "temperature1 "
         if (not validTemperature2Range):
            errSensor = errSensor + "temperature2 "

         log(GLB.DEBUG_SAVE, "Error, " + errSensor + "data out of range\n")
         log(GLB.debugType, "Error, " + errSensor + "data out of range\n")
   
   return

#store info into file
def storeToFile(fileName, data):
   myFileOutput = open(fileName, "a")
   myFileOutput.write(data)
   myFileOutput.close()
   return

#create log file path if it does not exist
def createLogFilePath():
   #if directory battlog does not exist, create one
   if (not(os.path.isdir(GLB.LOG_FILE_PATH))):
        os.mkdir(GLB.LOG_FILE_PATH)
   return  

#create rtc log file path if it does not exist
def createRTCFilePath():
   #if directory rtclog does not exist, create one
   if (not(os.path.isdir(GLB.RTC_FILE_PATH))):
        os.mkdir(GLB.RTC_FILE_PATH)
   return

#create debug log file path if it does not exist
def createDebugFilePath():
   #if directory rtclog does not exist, create one
   if (not(os.path.isdir(GLB.DEBUG_FILE_PATH))):
        os.mkdir(GLB.DEBUG_FILE_PATH)
   return

#get current time in a string
def getTimeString():
   return time.strftime("%m%d%y%H%M%S")

#get current time in a string formatted for reading
def getFormatTimeString():
   return time.strftime("%m/%d/%y %H:%M:%S")

#get new file name based on current time
def getNewFileName():
   return (GLB.LOG_FILE_NAME+getTimeString()+".txt")

#get new file name based on current time
def getDebugNewFileName():
   return (GLB.DEBUG_FILE_NAME+getTimeString()+".csv")

#use this function to print or save debug msg
def log(logType, msg):
   if (logType==GLB.DEBUG_DETAIL):
      print msg
   elif (logType==GLB.DEBUG_SAVE):
      storeToFile(GLB.DEBUG_LOG_FILE_NAME, getFormatTimeString() + " " + msg)

#use this function to filter out of range data
def verfiyDataRange(dataType, value):
   goodRange = False
   #check boundaries of voltage
   if (dataType==GLB.DATA_TYPE_VOLTAGE):
      if (value<=GLB.MAX_VOLTAGE):
         goodRange = True
   #check boundaries of current
   elif (dataType==GLB.DATA_TYPE_CURRENT):
      if (math.fabs(value)<=GLB.MAX_ABS_CURRENT):
         goodRange = True
   #check boundaries of temperature
   elif (dataType==GLB.DATA_TYPE_TEMPERATURE):
      if (value>=GLB.MIN_TEMP and value<=GLB.MAX_TEMP):
         goodRange = True
   return goodRange
   

