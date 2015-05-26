import globals as GLB
import time, spidev, os, sys, math
from readtemp import ReadTemp
from datetime import datetime
import RPi.GPIO as GPIO # Import GPIO library
import threading
import time


class PICReader(threading.Thread):

    stop = None
    currentSecond = None
    newSecond = None
    v1reference = 0
    v2reference = 0
    v3reference = 0
    c1reference = 0
    c2reference = 0

    def __init__(self):
        super(PICReader, self).__init__()
        self.stop = threading.Event()
        self.createDebugFilePath()
        self.createLogFilePath()
        self.initSPI()
        self.setupPIC24RTC()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GLB.CALIBRATION_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def run(self):
##        if GPIO.input(GLB.CALIBRATION_PIN):
 ##           self.log(GLB.debugType, "Calibration initiating...")
  ##          self.calibrate()
   ##         self.log(GLB.debugType, "Calibration complete")
    ##    else:
          self.initialize_calibrated_values()
          self.collectTimeStampData()

    def join(self, timeout=None):
        self.stop.set()
        self.closeSPI()
        super(PICReader, self).join(timeout)

    # for handling of script inputs
    def getSPIOutputByte(self, input):
       msb = input >> 8
       lsb = input & 0xff
       GLB.spi.writebytes([msb, lsb])
       data = GLB.spi.readbytes(2)
       return data

    def get16bitSPIData(self, returnData):
       return self.getWord(returnData[0], returnData[1])

    def getWord(self, msb, lsb):
        return msb*256+lsb

    # convert decimal to hex value
    def decToHex(self, decValue):
        firstDigit = (decValue / 10) << 4
        secondDigit = decValue % 10
        return firstDigit + secondDigit

    def getSystemTimeInHex(self):
        rawTimeData = (time.strftime("%y/%m/%d/%H/%M/%S")).split("/")
        for i in range(0, 6):
          rawTimeData[i] = self.decToHex(int(rawTimeData[i]))
        return rawTimeData

    def addSPIDataMarking(self, sendData):
        return sendData | 0x8000

    def removeSPIDataMarking(self, receiveData):
        return receiveData & 0x7fff

    def sendSPIDataWithMarking(self, SPIOutput):
        return self.getSPIOutputByte(self.addSPIDataMarking(SPIOutput))

    def createCommandData(self, commandType, rawData):
        return (commandType << 8) + rawData

    def getPinReading(self, pin):
        my16bitSPIData = self.get16bitSPIData(self.sendSPIDataWithMarking(self.createCommandData(pin, GLB.NULL)))
        return self.removeSPIDataMarking(my16bitSPIData)

    # get current value
    def getCurrReading(self, voltVal, multiplier):
        voltVal *= multiplier
        if voltVal < GLB.iref:
            currentVal = (GLB.iref - voltVal) * -62.5
        else:
            currentVal = (voltVal - GLB.iref) * 62.5
        return round(currentVal, GLB.DECIMAL_ACCURACY)

    # get voltage value
    def getVoltageReading(self, voltVal, multiplier):
        voltVal *= multiplier
        if voltVal < GLB.vref:
            voltageVal = (GLB.vref - voltVal)
        else:
            voltageVal = voltVal - GLB.vref
        return round(voltageVal, GLB.DECIMAL_ACCURACY)

    def getTempReading(self, tempVal):
        return round(tempVal, GLB.DECIMAL_ACCURACY)

    # get last 6 digit of mac and use it as an id
    def getDeviceId(self):
        # Read MAC from file
        myMAC = open('/sys/class/net/eth0/address').read()
        macToken = myMAC.split(":")
        uniqueId = macToken[3] + macToken[4] + macToken[5]
        return uniqueId.replace("\n", "")

    # use this function to reset pic24
    def resetPic24(self):
        self.initPicResetGPIO()
        GPIO.output(GLB.NRESET_PIC24_GPIO_PIN, False)
        time.sleep(GLB.NRESET_PIC24_HOLD_TIME)
        GPIO.output(GLB.NRESET_PIC24_GPIO_PIN, True)
        self.releaseGPIO()
        return

    # use this function to reset entire system
    def resetSystem(self):
        self.initNPorSysGPIO()
        GPIO.output(GLB.NPOR_SYS_GPIO_PIN, False)
        time.sleep(GLB.NPOR_SYS_HOLD_TIME)
        GPIO.output(GLB.NPOR_SYS_GPIO_PIN, True)
        self.releaseGPIO()
        return

    # open connection and setup spi so we can talk to the pic
    def initSPI(self):
        # GLB.spi = spidev.SpiDev()
        GLB.spi.open(0, 0)
        return

    # close spi connection
    def closeSPI(self):
        GLB.spi.close()
        return

    # init pic reset gpio
    def initPicResetGPIO(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM) ## Use bcm numbering
        GPIO.setup(GLB.NRESET_PIC24_GPIO_PIN, GPIO.OUT) ## Setup NRESET_PIC24_PIN as output
        return

    # init npor gpio
    def initNPorSysGPIO(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM) ## Use board pin numbering
        GPIO.setup(GLB.NPOR_SYS_GPIO_PIN, GPIO.OUT) ## Setup NPOR_SYS_PIN as output
        return

    # release gpio pins
    def releaseGPIO(self):
        GPIO.cleanup()
        return

    def calibrate(self):
        tsData = []

        for i in range(GLB.GET_RTC_YEAR, GLB.GET_ADC_DATA6+1):
            data = self.getPinReading(i)
            tsData.append(data)

        v1Reading = GLB.REFERENCE_VOLTAGE - self.getVoltageReading(tsData[GLB.TS_DATA_V1], GLB.ADC_VOLT3vRATIO)
        v2Reading = GLB.REFERENCE_VOLTAGE - self.getVoltageReading(tsData[GLB.TS_DATA_V2], GLB.ADC_VOLT3vRATIO)
        v3Reading = GLB.REFERENCE_VOLTAGE - self.getVoltageReading(tsData[GLB.TS_DATA_V3], GLB.ADC_VOLT3vRATIO)
        c1Reading = GLB.REFERENCE_CURRENT - self.getCurrReading(tsData[GLB.TS_DATA_C1], GLB.ADC_3_3V_RATIO)
        c2Reading = GLB.REFERENCE_CURRENT - self.getCurrReading(tsData[GLB.TS_DATA_C2], GLB.ADC_3_3V_RATIO)
        collectedData = str(v1Reading) + "," + str(v2Reading) + "," + str(v3Reading) + "," + \
            str(c1Reading) + "," + str(c2Reading) + "\n"
        self.storeToFile(GLB.CALIB_FILE_NAME, collectedData, False)

    def initialize_calibrated_values(self):
        try:
            with open(GLB.CALIB_FILE_NAME) as cfile:
                line = cfile.readline()
                cfile.close()
                cdata = line.split(",", 1)
                if len(cdata) == 5:
                    self.v1reference = cdata[0]
                    self.v2reference = cdata[1]
                    self.v3reference = cdata[2]
                    self.c1reference = cdata[3]
                    self.c2reference = cdata[4]
        except Exception, e:
            self.log(GLB.debugType, "Exception while reading calibration file %s:%s" % (GLB.CALIB_FILE_NAME, e))

    # collect time stamp data
    def collectTimeStampData(self):

        dataCount=0
        currentFileName=GLB.LOG_FILE_NAME
        rtemp = ReadTemp()

        # scan pic every 1 sec for new adc data until stopped
        while not self.stop.isSet():

            # get a new file name in the beginning of the data collection cycle
            if dataCount == 0:
                currentFileName = self.getNewFileName()

            now = datetime.now()
            picTime = now.strftime("%Y-%m-%d %H:%M:%S")
            self.currentSecond = now.second

            tsData = []

            for i in range(GLB.GET_RTC_YEAR, GLB.GET_ADC_DATA6+1):
                data = self.getPinReading(i)
                tsData.append(data)

            v1Reading = self.getVoltageReading(tsData[GLB.TS_DATA_V1], GLB.ADC_VOLT3vRATIO) + self.v1reference
            v2Reading = self.getVoltageReading(tsData[GLB.TS_DATA_V2], GLB.ADC_VOLT3vRATIO) + self.v2reference
            v3Reading = self.getVoltageReading(tsData[GLB.TS_DATA_V3], GLB.ADC_VOLT3vRATIO) + self.v3reference
            c1Reading = self.getCurrReading(tsData[GLB.TS_DATA_C1], GLB.ADC_3_3V_RATIO) + self.c1reference
            c2Reading = self.getCurrReading(tsData[GLB.TS_DATA_C2], GLB.ADC_3_3V_RATIO) + self.c2reference
            t1Reading = ""
            t2Reading = ""
            tempc = rtemp.read_temp()
            if "TR1" in tempc:
                t1Reading = self.getTempReading(tempc["TR1"])
            if "TR2" in tempc:
                t2Reading = self.getTempReading(tempc["TR2"])

            # discard duplicates
            if self.is_dup():
                continue

            collectedData = picTime + "," + self.getDeviceId() + "," + \
            str(v1Reading) + "," + str(v2Reading) + "," + str(v3Reading) + "," + \
            str(c1Reading) + "," + str(c2Reading) + "," + \
            str(t1Reading) + "," + str(t2Reading) + "\n"

            # check if data pass range test
            #validVoltage1Range = self.verfiyDataRange(GLB.DATA_TYPE_VOLTAGE, float(v1Reading))
            #validVoltage2Range = self.verfiyDataRange(GLB.DATA_TYPE_VOLTAGE, float(v2Reading))
            #validVoltage3Range = self.verfiyDataRange(GLB.DATA_TYPE_VOLTAGE, float(v3Reading))
            #validCurrent1Range = self.verfiyDataRange(GLB.DATA_TYPE_CURRENT, float(c1Reading))
            #validCurrent2Range = self.verfiyDataRange(GLB.DATA_TYPE_CURRENT, float(c2Reading))
            #validTemperature1Range = self.verfiyDataRange(GLB.DATA_TYPE_TEMPERATURE, float(t1Reading))
            #validTemperature2Range = self.verfiyDataRange(GLB.DATA_TYPE_TEMPERATURE, float(t2Reading))

            #validAllRange = (validVoltage1Range and validVoltage2Range and validVoltage3Range and \
            #validCurrent1Range and validCurrent2Range and \
            #validTemperature1Range and validTemperature2Range)

            # only store values if all range pass test
            #if validAllRange:

            # store data
            self.storeToFile(currentFileName, collectedData)

            self.log(GLB.debugType, "[TIME] " + picTime)
            self.log(GLB.debugType, "[ID] " + self.getDeviceId())
            self.log(GLB.debugType, "[V1] " + str(v1Reading))
            self.log(GLB.debugType, "[V2] " + str(v2Reading))
            self.log(GLB.debugType, "[V3] " + str(v3Reading))
            self.log(GLB.debugType, "[C1] " + str(c1Reading))
            self.log(GLB.debugType, "[C2] " + str(c2Reading))
            self.log(GLB.debugType, "[T1] " + str(t1Reading))
            self.log(GLB.debugType, "[T2] " + str(t2Reading) + "\n")

            #else:
             #   self.log(GLB.debugType, "Invalid ranges detected")

            dataCount += 1
            if dataCount > GLB.TS_DURATION:
                dataCount = 0
        return

    def is_dup(self):
        dup = False
        self.newSecond = datetime.now().second
        if self.currentSecond == self.newSecond:
            dup = True
            self.log(GLB.debugType, "Duplicate reading found at second " + str(self.newSecond) + "\n")
        self.currentSecond = self.newSecond
        return dup

    # store info into file
    def storeToFile(self, fileName, data, append=True):
        if append:
            myFileOutput = open(fileName, "a")
        else:
            myFileOutput = open(fileName, "w")
        myFileOutput.write(data)
        myFileOutput.close()
        return

    # create log file path if it does not exist
    def createLogFilePath(self):
        # if directory battlog does not exist, create one
        if not(os.path.isdir(GLB.LOG_FILE_PATH)):
            os.mkdir(GLB.LOG_FILE_PATH)
        return

    # create rtc log file path if it does not exist
    def createRTCFilePath(self):
        # if directory rtclog does not exist, create one
        if not(os.path.isdir(GLB.RTC_FILE_PATH)):
            os.mkdir(GLB.RTC_FILE_PATH)
        return

    # create debug log file path if it does not exist
    def createDebugFilePath(self):
        # if directory rtclog does not exist, create one
        if not(os.path.isdir(GLB.DEBUG_FILE_PATH)):
            os.mkdir(GLB.DEBUG_FILE_PATH)
        return

    # get current time in a string
    def getTimeString(self):
        return time.strftime("%m%d%y%H%M%S")

    # get current time in a string formatted for reading
    def getFormatTimeString(self):
        return time.strftime("%m/%d/%y %H:%M:%S")

    # get new file name based on current time
    def getNewFileName(self):
        return GLB.LOG_FILE_NAME+self.getTimeString()+".txt"

    # get new file name based on current time
    def getDebugNewFileName(self):
        return GLB.DEBUG_FILE_NAME+self.getTimeString()+".csv"

    # use this function to print or save debug msg
    def log(self, logType, msg):
       if logType == GLB.DEBUG_DETAIL:
           self.storeToFile(GLB.DEBUG_LOG_FILE_NAME, self.getFormatTimeString() + " " + msg)
       elif logType == GLB.DEBUG_SAVE:
           self.storeToFile(GLB.DEBUG_LOG_FILE_NAME, self.getFormatTimeString() + " " + msg)

    # use this function to filter out of range data
    def verfiyDataRange(self, dataType, value):
        goodRange = False
        # check boundaries of voltage
        if dataType == GLB.DATA_TYPE_VOLTAGE:
            if value <= GLB.MAX_VOLTAGE:
                goodRange = True
        # check boundaries of current
        elif (dataType==GLB.DATA_TYPE_CURRENT):
            if math.fabs(value) <= GLB.MAX_ABS_CURRENT:
                goodRange = True
        # check boundaries of temperature
        elif dataType == GLB.DATA_TYPE_TEMPERATURE:
            if value >= GLB.MIN_TEMP and value <= GLB.MAX_TEMP:
                goodRange = True
        return goodRange

    # set up rtc time in pic
    def setupPIC24RTC(self):

        # init retry attempt counter
        retryAttempt = 0

        currentPICResetCount = 0

        # get sys time in hex
        sysTimeHex = self.getSystemTimeInHex()
        rtcSetupDone = False

        while not rtcSetupDone:
            for i in range(GLB.SET_RTC_YEAR, GLB.SET_RTC_SECOND+1):
                my16bitSPIData = self.get16bitSPIData(self.sendSPIDataWithMarking(self.createCommandData(i, sysTimeHex[i])))
                SPIAck = self.removeSPIDataMarking(my16bitSPIData)

                # no ack from pic that we set up rtc
                if not SPIAck:
                    retryAttempt = retryAttempt + 1
                    self.log(GLB.debugType, "rtc variable " + str(i) + " set up failed, retry setup in 1 second, attemp #" + str(retryAttempt) + "\n")
                    time.sleep(GLB.SETUP_FAILED_DELAY)

                    # reset pic if max number of retry attempts is reached
                    if retryAttempt>=GLB.MAX_RETRY_ATTEMPT:

                        # reset system if max number of resets on pic still does not resolve issue
                        if (currentPICResetCount>=GLB.MAX_PIC_RESET):
                            self.log(GLB.debugType, "max number of pic reset for rtc setup reached, resetting system\n")
                            # catastrophic failure, reset entire board and exit program
                            self.resetSystem()
                            sys.exit()

                        # reset pic to see if we can fix the rtc setup issue
                        else:
                            self.log(GLB.debugType, "max number of retry attempt for rtc setup is reached, resetting pic\n")
                            self.resetPic24()
                            time.sleep(GLB.NRESET_PIC24_HOLD_TIME*2)
                            retryAttempt = 0
                            currentPICResetCount = currentPICResetCount + 1
                    break

                # rtc variable got set up correctly
                self.log(GLB.debugType, "rtc variable " + str(i) + " set up correctly\n")

                # rtc setup is done
                if i == GLB.SET_RTC_SECOND:
                    self.log(GLB.debugType, "rtc set up successful\n")
                    rtcSetupDone = True
                    time.sleep(GLB.PIC_SETUP_DELAY)
        return
