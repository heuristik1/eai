import time, spidev, os, sys, math
import globals as GLB
from readtemp import ReadTemp
import RPi.GPIO as GPIO # Import GPIO library
import threading


class PICReader(threading.Thread):

    stop = None

    def __init__(self):
        super(PICReader, self).__init__()
        self.stop = threading.Event()
        self.createDebugFilePath()
        self.createLogFilePath()
        self.initSPI()

    def run(self):
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

    # get approximate values of temperature reading based on equation
    def getTempReading(self, voltVal):
        fahrenheitVal = (-4.71728 * voltVal**7) + (57.51 * voltVal**6) + \
          (-290.91 * voltVal**5) + (789.2 * voltVal**4) + \
          (-1247.4 * voltVal**3) + (1176.45 * voltVal**2) + \
          (-704.13 * voltVal) + 343.992
        celciusVal = self.fahrenheitToCelcius(fahrenheitVal)
        return celciusVal

    # convert fahrenheit to celcius
    def fahrenheitToCelcius(self, fahrenheightVal):
        return (fahrenheightVal-32.0) * (5.0/9.0)

    # get current value
    def getCurrReading(self, voltVal):
        currentVal = (voltVal - GLB.vref) * 125
        if voltVal<GLB.vref:
            currentVal = (GLB.vref - voltVal) * -125
        return currentVal

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

    # find vref
    def findVref(self):
        myMacAddress = self.getDeviceId()
        if myMacAddress == GLB.DEVICE_ID_816874:
            GLB.vref = GLB.MAC_ADDRESS_816874_VREF
        elif myMacAddress == GLB.DEVICE_ID_8daf22:
            GLB.vref = GLB.MAC_ADDRESS_8daf22_VREF
        elif myMacAddress == GLB.DEVICE_ID_8ece5a:
            GLB.vref = GLB.MAC_ADDRESS_8ece5a_VREF
        else:
            GLB.vref = GLB.MAC_ADDRESS_OTHER_VREF
        return

    # collect time stamp data
    def collectTimeStampData(self):

        # find vref
        self.findVref()
        self.log(GLB.debugType, "vref = " + str(GLB.vref) + "\n")

        # scan pic every 1 sec for new adc data until stopped
        while not self.stop.isSet():
            # get time stamp data in pic
            TSdata = []

            # get a new file name in the beginning of the data collection cycle
            if dataCount == 0:
                currentFileName = self.getNewFileName()
                #for debugging
                curentDebugFileName = self.getDebugNewFileName()

            for i in range(GLB.GET_RTC_YEAR, GLB.GET_ADC_DATA6+1):
                my16bitSPIData = self.get16bitSPIData(self.sendSPIDataWithMarking(self.createCommandData(i, GLB.NULL)))
                SPIData = self.removeSPIDataMarking(my16bitSPIData)
                TSdata.append(SPIData)

            picTime = time.strftime("%Y-%m-%d %H:%M:%S")
            rtemp = ReadTemp()
            tempc = rtemp.read_temp()
            v1Reading = str(round(TSdata[GLB.TS_DATA_V1] * GLB.ADC_3_3V_RATIO * GLB.VOLTAGE_ADC_RATIO, GLB.DECIMAL_ACCURACY))
            v2Reading = str(round(TSdata[GLB.TS_DATA_V2] * GLB.ADC_3_3V_RATIO * GLB.VOLTAGE_ADC_RATIO, GLB.DECIMAL_ACCURACY))
            v3Reading = str(round(TSdata[GLB.TS_DATA_V3] * GLB.ADC_3_3V_RATIO * GLB.VOLTAGE_ADC_RATIO, GLB.DECIMAL_ACCURACY))
            t1Reading = ""
            t2Reading = ""
            try:
                t1Reading = str(round(tempc["TR1"], GLB.DECIMAL_ACCURACY))
                t2Reading = str(round(tempc["TR2"], GLB.DECIMAL_ACCURACY))
            except Exception:
                self.log(GLB.debugType , "Unable to gather temperature reading from TR1,TR2/n")

            c1Reading = str(round(self.getCurrReading(TSdata[GLB.TS_DATA_C1] * GLB.ADC_3_3V_RATIO), GLB.DECIMAL_ACCURACY))
            c2Reading = str(round(self.getCurrReading(TSdata[GLB.TS_DATA_C2] * GLB.ADC_3_3V_RATIO), GLB.DECIMAL_ACCURACY))

            collectedData = picTime + "," + self.getDeviceId() + "," + \
            v1Reading + "," + v2Reading + "," + v3Reading + "," + \
            c1Reading + "," + c2Reading + "," + \
            t1Reading + "," + t2Reading + "\n"

            # check if data pass range test
            validVoltage1Range = self.verfiyDataRange(GLB.DATA_TYPE_VOLTAGE, float(v1Reading))
            validVoltage2Range = self.verfiyDataRange(GLB.DATA_TYPE_VOLTAGE, float(v2Reading))
            validVoltage3Range = self.verfiyDataRange(GLB.DATA_TYPE_VOLTAGE, float(v3Reading))
            validCurrent1Range = self.verfiyDataRange(GLB.DATA_TYPE_CURRENT, float(c1Reading ))
            validCurrent2Range = self.verfiyDataRange(GLB.DATA_TYPE_CURRENT, float(c2Reading ))
            validTemperature1Range = self.verfiyDataRange(GLB.DATA_TYPE_TEMPERATURE, float(t1Reading))
            validTemperature2Range = self.verfiyDataRange(GLB.DATA_TYPE_TEMPERATURE, float(t2Reading))

            validAllRange = (validVoltage1Range and validVoltage2Range and validVoltage3Range and \
            validCurrent1Range and validCurrent2Range and \
            validTemperature1Range and validTemperature2Range)

            # only store values if all range pass test
            if validAllRange:
                currentSecond = int(TSdata[GLB.TS_DATA_SECOND])
                identicalSecondFound = False

            if not identicalSecondFound:
                dataCount += 1

                # store data
                self.storeToFile(currentFileName, collectedData)

                #for debugging purposes
                #storeToFile(curentDebugFileName, collectedData)

                # reset counter
                if dataCount >= GLB.TS_DURATION:
                    dataCount = 0

                self.log(GLB.debugType, "[TIME] " + picTime)
                self.log(GLB.debugType, "[ID] " + self.getDeviceId())
                self.log(GLB.debugType, "[V1] " + v1Reading)
                self.log(GLB.debugType, "[V2] " + v2Reading)
                self.log(GLB.debugType, "[V3] " + v3Reading)
                self.log(GLB.debugType, "[C1] " + c1Reading)
                self.log(GLB.debugType, "[C2] " + c2Reading)
                self.log(GLB.debugType, "[T1] " + t1Reading)
                self.log(GLB.debugType, "[T2] " + t2Reading + "\n")

                # set delay for calibrated time
                # time.sleep(GLB.TS_COLLECTION_DELAY)
                time.sleep(1)

            else:
                # try reset spi module on pi to fix problem
                self.closeSPI()
                self.initSPI()

                errSensor = ""
                if not validVoltage1Range:
                    errSensor = "voltage1 "
                if not validVoltage2Range:
                    errSensor = errSensor + "voltage2 "
                if not validVoltage3Range:
                    errSensor = errSensor + "voltage3 "
                if not validCurrent1Range:
                    errSensor = errSensor + "current1 "
                if not validCurrent2Range:
                    errSensor = errSensor + "current2 "
                if not validTemperature1Range:
                    errSensor = errSensor + "temperature1 "
                if not validTemperature2Range:
                    errSensor = errSensor + "temperature2 "

                self.log(GLB.DEBUG_SAVE, "Error, " + errSensor + "data out of range\n")
                self.log(GLB.debugType, "Error, " + errSensor + "data out of range\n")
        return

    # store info into file
    def storeToFile(self, fileName, data):
        myFileOutput = open(fileName, "a")
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
            print msg
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

