import RPi.GPIO as GPIO
import time
from datetime import datetime
from RPLCD.i2c import CharLCD

# for LCD setup (adjust address if needed after i2cdetect)

lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1,
              cols=16, rows=2, charmap='A00')

def lcdWrite(msg, line=0):
    lcd.clear()
    lcd.cursor_pos = (line, 0)
    lcd.write_string(msg)


# for rtc

class DS1302:
    def __init__(self, ce, data, sclk, debug=False):
        self.ce = ce
        self.data = data
        self.sclk = sclk
        self.debug = debug

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ce, GPIO.OUT)
        GPIO.setup(self.sclk, GPIO.OUT)
        GPIO.setup(self.data, GPIO.OUT)
        
    # Low-level helpers
    def _write_byte(self, byte):
        for i in range(8):
            GPIO.output(self.data, (byte >> i) & 1)
            GPIO.output(self.sclk, 1)
            GPIO.output(self.sclk, 0)

    def _read_byte(self):
        GPIO.setup(self.data, GPIO.IN)
        value = 0
        for i in range(8):
            bit = GPIO.input(self.data)
            value |= (bit << i)
            GPIO.output(self.sclk, 1)
            GPIO.output(self.sclk, 0)
        GPIO.setup(self.data, GPIO.OUT)
        return value

    def _bcd_to_dec(self, bcd):
        return (bcd >> 4) * 10 + (bcd & 0x0F)

    def _dec_to_bcd(self, dec):
        return ((dec // 10) << 4) | (dec % 10)

    # High-level methods
    def read_time(self):
        GPIO.output(self.ce, 1)
        self._write_byte(0xBF)  # Burst read command

        raw = [self._read_byte() for _ in range(8)]
        GPIO.output(self.ce, 0)

        if self.debug:
            print("[DEBUG] Raw registers:", [hex(x) for x in raw])
            print("[DEBUG] Decoded:",
                  "sec=", self._bcd_to_dec(raw[0] & 0x7F),
                  "min=", self._bcd_to_dec(raw[1] & 0x7F),
                  "hour=", self._bcd_to_dec(raw[2] & 0x3F),
                  "date=", self._bcd_to_dec(raw[3] & 0x3F),
                  "month=", self._bcd_to_dec(raw[4] & 0x1F),
                  "year=", self._bcd_to_dec(raw[6]) + 2000)

        sec    = self._bcd_to_dec(raw[0] & 0x7F)
        minute = self._bcd_to_dec(raw[1] & 0x7F)
        hour   = self._bcd_to_dec(raw[2] & 0x3F)  # 24-hour mode
        date   = self._bcd_to_dec(raw[3] & 0x3F)
        month  = self._bcd_to_dec(raw[4] & 0x1F)
        year   = self._bcd_to_dec(raw[6]) + 2000

        return datetime(year, month, date, hour, minute, sec)

    def set_time(self, year, month, date, hour, minute, sec):
        GPIO.output(self.ce, 1)
        self._write_byte(0xBE)  # Burst write command

        self._write_byte(self._dec_to_bcd(sec) & 0x7F)
        self._write_byte(self._dec_to_bcd(minute) & 0x7F)
        self._write_byte(self._dec_to_bcd(hour) & 0x3F)
        self._write_byte(self._dec_to_bcd(date) & 0x3F)
        self._write_byte(self._dec_to_bcd(month) & 0x1F)
        self._write_byte(self._dec_to_bcd(0))   # weekday
        self._write_byte(self._dec_to_bcd(year % 100))
        self._write_byte(0x00)  # control register

        GPIO.output(self.ce, 0)

# custom functions

# check time, if match return true
# @params: num, num, num: xx, xx, xx format
def checkTime(hour, minute, second):
    now = rtc.read_time()
    return (hour == now.hour and minute == now.minute and second == now.second)

# @params: angle from -90 to 90
# assume every call of this func a med will be dispensed
def moveServoAngle():
	pwm.ChangeDutyCycle(7.8)
	time.sleep(0.35)
	pwm.ChangeDutyCycle(0)
	
# for movement
def forward():
	print("forward")
	
def rotate360():
	print("360")
	
def stop():
	print("stop")
	
# for each med loop test
def testServo():
	moveServoAngle()
	print("1")
	time.sleep(2)
	moveServoAngle()
	print("2")
	time.sleep(2)
	moveServoAngle()
	print("3")
	time.sleep(2)
	moveServoAngle()
	print("4")
	time.sleep(2)
	moveServoAngle()
	print("5")
	time.sleep(2)
	moveServoAngle()
	print("6")
	time.sleep(2)
	moveServoAngle()
	print("7")
	time.sleep(2)


# for ultrasonic sensor
def getDistance():
	GPIO.output(trigPin, True)
	time.sleep(0.00001)
	GPIO.output(trigPin, False)
	
	start_time = None
	end_time = None
	
	while GPIO.input(echoPin) == 0:
		start_time = time.time()
		
	while GPIO.input(echoPin) == 1:
		end_time = time.time()
	
	if start_time is not None and end_time is not None:
		duration = end_time - start_time
		distance = (duration * 34300) / 2
		return round(distance, 2)
	else:
		return None
		
# for queue dispensing of medicine
# arranged from early to late up to 7
arrSched = [
	{
		"hour": 1,
		"minute": 0,
		"second": 0,
		"medicine": "Paracetamol"
	},
	{
		"hour": 1,
		"minute": 2,
		"second": 0,
		"medicine": "Mefenamic"
	},
	{
		"hour": 1,
		"minute": 4,
		"second": 0,
		"medicine": "Ibuprofen"
	},
	{
		"hour": 1,
		"minute": 6,
		"second": 0,
		"medicine": "idk"
	},
	{
		"hour": 1,
		"minute": 8,
		"second": 0,
		"medicine": "idk"
	},
	{
		"hour": 1,
		"minute": 9,
		"second": 0,
		"medicine": "idk"
	},
	{
		"hour": 1,
		"minute": 10,
		"second": 0,
		"medicine": "idk"
	},
]
	
# debugging
def debugLog():
	# ultrasonic
	print(getDistance())
	time.sleep(.1)
	# rtc
	now = rtc.read_time()
	print(f"Current time: {now.hour:02d}:{now.minute:02d}:{now.second:02d}")

# GPIO set pins
GPIO.setmode(GPIO.BCM)

servoPin = 17
motorLeftPin = 5
motorRightPin = 6
buzzerPin = 10

# ultrasonic sensor pins
trigPin = 23
echoPin = 24

# setup
GPIO.setup(motorLeftPin, GPIO.OUT)
GPIO.setup(motorRightPin, GPIO.OUT)
GPIO.setup(buzzerPin, GPIO.OUT)
GPIO.setup(trigPin, GPIO.OUT)
GPIO.setup(echoPin, GPIO.IN)
GPIO.setup(servoPin, GPIO.OUT)

# pwm setup
pwm = GPIO.PWM(servoPin, 50)
pwm.start(0)

# threshold variables
distanceThreshold = 20

# rtc also
CLK_PIN = 25   # SCLK
DAT_PIN = 27   # I/O
RST_PIN = 22   # CE (RST)
rtc = DS1302(ce=RST_PIN, data=DAT_PIN, sclk=CLK_PIN, debug=False)

# other variables
medicineIndex = 0

# loop	
try:
	# set servo pwm
	while True:
		debugLog()
		# check if its time for medicine then if true, execute task
		if checkTime(arrSched[medicineIndex]["hour"], arrSched[medicineIndex]["minute"], arrSched[medicineIndex]["second"]):
			forward()
			if (getDistance() < distanceThreshold):
				stop()
				time.sleep(.5)
				moveServoAngle()
				lcdWrite(f"The medicine {arrSched[medicineIndex]['medicine']} is delivered.")
				GPIO.output(buzzerPin, True)
				time.sleep(.5)
				rotate360()
				GPIO.output(buzzerPin, False)
				time.sleep(.5)
				forward()
				if (getDistance() < distanceThreshold):
					rotate360()
					time.sleep(.5)
					stop()
					medicineIndex += 1
		
		if medicineIndex == len(arrSched):
			lcdWrite("all schedule finished")
			medicineIndex = 0
			
		
except KeyboardInterrupt:
	pwm.stop()
	GPIO.cleanup()
