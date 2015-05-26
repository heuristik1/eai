import spidev

# constants for pic communication protocal
SET_RTC_YEAR =       0
SET_RTC_MONTH =      1
SET_RTC_DAY =        2
SET_RTC_HOUR =       3
SET_RTC_MINUTE =     4
SET_RTC_SECOND =     5
PREPARE_TS =         6
GET_RTC_YEAR =       7
GET_RTC_MONTH =      8
GET_RTC_DAY =        9
GET_RTC_HOUR =       10
GET_RTC_MINUTE =     11
GET_RTC_SECOND =     12
GET_ADC_DATA0 =      13
GET_ADC_DATA1 =      14
GET_ADC_DATA2 =      15
GET_ADC_DATA3 =      16
GET_ADC_DATA4 =      17
GET_ADC_DATA5 =      18
GET_ADC_DATA6 =      19

# constants for pic setup
PIC_SETUP_DELAY =       .88
SETUP_FAILED_DELAY =    .1
# TS_COLLECTION_DELAY =   1.2250
# TS_COLLECTION_DELAY =   .886
TS_COLLECTION_DELAY =   .2
NULL =                  0
MIN_CALIBRATE_DECREMENT = .0001
CALIBRATION_PIN = 21

# constants for data manipulation
DECIMAL_ACCURACY =	4
VOLTAGE_ADC_RATIO =     30.4118
ADC_3_3V_RATIO =        0.0008058
ADC_VOLT3vRATIO=     0.02450582844

# constants for data index
TS_DATA_YEAR = 0
TS_DATA_MONTH = 1
TS_DATA_DAY = 2
TS_DATA_HOUR = 3
TS_DATA_MINUTE = 4
TS_DATA_SECOND = 5
TS_DATA_V1 = 6
TS_DATA_V2 = 7
TS_DATA_V3 = 8
TS_DATA_C1 = 9
TS_DATA_C2 = 10
TS_DATA_T1 = 11
TS_DATA_T2 = 12

# constants for debug
DEBUG_NONE = 0
DEBUG_MINIMUM = 1
DEBUG_DETAIL = 2
DEBUG_SAVE = 3

# constants for rasp layout
NRESET_PIC24_GPIO_PIN = 17
NPOR_SYS_GPIO_PIN = 7

# constants for reset control setup
NRESET_PIC24_HOLD_TIME = .1
NPOR_SYS_HOLD_TIME = .1

# constants for error handling
MAX_RETRY_ATTEMPT = 10
MAX_PIC_RESET = 600

# constants for data bound checking
DATA_TYPE_VOLTAGE = 0
DATA_TYPE_CURRENT = 1
DATA_TYPE_TEMPERATURE = 2
MAX_VOLTAGE = 60
MAX_ABS_CURRENT = 100
MAX_TEMP = 70
MIN_TEMP = -40

# constants for manual calibration of the vref, use for applying current equation
MAC_ADDRESS_816874_VREF = 1.6505 #ip .60
MAC_ADDRESS_8daf22_VREF = 1.6503 #ip .81
MAC_ADDRESS_8ece5a_VREF = 1.6485 #1.65, ip .84
MAC_ADDRESS_OTHER_VREF = 1.65
DEVICE_ID_816874 = "816874"
DEVICE_ID_8daf22 = "8daf22"
DEVICE_ID_8ece5a = "8ece5a"

# constants used for file manipulation
CALIB_FILE_NAME="/home/pi/calib/calibration"
LOG_FILE_NAME = "/home/pi/battlog/myLog"
DEBUG_FILE_NAME = "/home/pi/debuglog/longLog"
DEBUG_LOG_FILE_NAME = "/home/pi/debuglog/saveLog"
LOG_FILE_PATH = "/home/pi/battlog"
RTC_FILE_PATH = "/home/pi/rtclog"
DEBUG_FILE_PATH = "/home/pi/debuglog"
SECONDS_IN_MINUTE = 60 #60 seconds
SECONDS_IN_HOUR = 3600 #3600 seconds
SECONDS_IN_12HOURS = 43200 #43200 seconds
SECONDS_IN_DAY = 86400 #86400 seconds

# set the data collection period to be every 12 hours
TS_DURATION = SECONDS_IN_MINUTE

# variables used for file manipulation and debug purposes
# dataCount = 0
# currentFileName = LOG_FILE_NAME
debugType = DEBUG_SAVE
debugType = DEBUG_DETAIL
# currentPICResetCount = 0

# variable used for spi communication
spi = spidev.SpiDev()

# variable used to store vref
vref = 0.00

# variable used to store iref
iref = 1.65

# Calibration Mode Variables
CALIBRATE_LOG = "/home/pi/calibrate"
CAL_LOG_NAME="home/pi/calibrate/cal_log"
REFERENCE_VOLTAGE = 10
REFERENCE_CURRENT = 0

