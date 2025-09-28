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
	# def read_time(self):
		# GPIO.output(self.ce, 1)
		# self._write_byte(0xBF)  # Burst read command

		# raw = [self._read_byte() for _ in range(8)]
		# GPIO.output(self.ce, 0)

		# sec    = self._bcd_to_dec(raw[0] & 0x7F)
		# minute = self._bcd_to_dec(raw[1] & 0x7F)
		# hour   = self._bcd_to_dec(raw[2] & 0x3F)  # 24-hour mode
		# date   = self._bcd_to_dec(raw[3] & 0x3F)
		# month  = self._bcd_to_dec(raw[4] & 0x1F)
		# year   = self._bcd_to_dec(raw[6]) + 2000

		# return datetime(year, month, date, hour, minute, sec)
	def read_time(self):
		GPIO.output(self.ce, 1)
		self._write_byte(0xBF)  # Burst read command

		raw = [self._read_byte() for _ in range(8)]
		GPIO.output(self.ce, 0)

		# print("RTC RAW:", raw)  # <--- ADD THIS

		try:
			sec    = self._bcd_to_dec(raw[0] & 0x7F)
			minute = self._bcd_to_dec(raw[1] & 0x7F)
			hour   = self._bcd_to_dec(raw[2] & 0x3F)
			date   = self._bcd_to_dec(raw[3] & 0x3F)
			month  = self._bcd_to_dec(raw[4] & 0x1F)
			year   = self._bcd_to_dec(raw[6]) + 2000

			# safety clamp
			if not 1 <= month <= 12: month = 1
			if not 1 <= date  <= 31: date  = 1
			if not 0 <= hour  <= 23: hour  = 0
			if not 0 <= minute<= 59: minute= 0
			if not 0 <= sec   <= 59: sec   = 0

			return datetime(year, month, date, hour, minute, sec)
		except Exception as e:
			print("RTC DECODE ERROR:", e)
			return datetime(2000, 1, 1, 0, 0, 0)  # fallback to safe default


# custom functions

# check time, if match return true
def checkTime(hour, minute, second):
	now = rtc.read_time()
	return (hour == now.hour and minute == now.minute and second == now.second)

# assume every call of this func a med will be dispensed
def moveServoAngle():
	pwm.ChangeDutyCycle(7.8)
	time.sleep(0.35)
	pwm.ChangeDutyCycle(0)
	
# for movement
def forward():
	print("forward")
	GPIO.output(motorLeftPin1, False)
	GPIO.output(motorLeftPin2, True)
	GPIO.output(motorRightPin1, False)
	GPIO.output(motorRightPin2, True)
	
def rotate360():
	print("360")
	GPIO.output(motorLeftPin1, True)
	GPIO.output(motorLeftPin2, False)
	GPIO.output(motorRightPin1, False)
	GPIO.output(motorRightPin2, True)
	
def stop():
	GPIO.output(motorLeftPin1, True)
	GPIO.output(motorLeftPin2, True)
	GPIO.output(motorRightPin1, True)
	GPIO.output(motorRightPin2, True)
	print("stop")

# for ultrasonic sensor
def getDistance():
	GPIO.output(trigPin, True)
	time.sleep(0.00001)
	GPIO.output(trigPin, False)

	start_time = time.time()
	timeout = start_time + 0.02

	while GPIO.input(echoPin) == 0 and time.time() < timeout:
		start_time = time.time()

	timeout = time.time() + 0.02
	while GPIO.input(echoPin) == 1 and time.time() < timeout:
		end_time = time.time()

	try:
		duration = end_time - start_time
		distance = (duration * 34300) / 2
		return round(distance, 2)
	except:
		return None
		
# for queue dispensing of medicine
arrSched = [
	{"hour": 16, "minute": 15, "second": 30, "medicine": "Paracetamol"},
	{"hour": 1,  "minute": 2,  "second": 0, "medicine": "Mefenamic"},
	{"hour": 1,  "minute": 4,  "second": 0, "medicine": "Ibuprofen"},
	{"hour": 1,  "minute": 6,  "second": 0, "medicine": "idk"},
	{"hour": 1,  "minute": 8,  "second": 0, "medicine": "idk"},
	{"hour": 1,  "minute": 9,  "second": 0, "medicine": "idk"},
	{"hour": 1,  "minute": 10, "second": 0, "medicine": "idk"},
]
	
# debugging
def debugLog():
	now = rtc.read_time()
	print(f"Current time: {now.hour:02d}:{now.minute:02d}:{now.second:02d} ultrasonic:{getDistance()}")
	lcdWrite(f"{now.hour:02d}:{now.minute:02d}:{now.second:02d}")
	# lcdWrite("Hello World")


# GPIO set pins
GPIO.setmode(GPIO.BCM)

servoPin = 17
motorLeftPin1 = 12
motorLeftPin2 = 16
motorRightPin1 = 13
motorRightPin2 = 26
buzzerPin = 10

# ultrasonic sensor pins
trigPin = 23
echoPin = 24

# setup
GPIO.setup(motorLeftPin1, GPIO.OUT)
GPIO.setup(motorLeftPin2, GPIO.OUT)
GPIO.setup(motorRightPin1, GPIO.OUT)
GPIO.setup(motorRightPin2, GPIO.OUT)
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
rtc = DS1302(ce=RST_PIN, data=DAT_PIN, sclk=CLK_PIN, debug=True)

# other variables
medicineIndex = 0
itsTime = False

# loop	
try:
	while True:
		debugLog()
		# forward()
		sched = arrSched[medicineIndex]

		# if time matches, set flag
		if checkTime(sched["hour"], sched["minute"], sched["second"]):
			itsTime = True

		# if flag is set, execute once
		if itsTime:
			lcd.clear()
			forward()
			time.sleep(1)
			dist = getDistance()
			# if dist is not None and dist < distanceThreshold:
			stop()
			time.sleep(.5)
			moveServoAngle()
			lcdWrite(f"{sched['medicine']} is delivered.")
			GPIO.output(buzzerPin, True)
			time.sleep(.5)
			rotate360()
			time.sleep(1)
			GPIO.output(buzzerPin, False)
			time.sleep(.5)
			forward()
			time.sleep(1)
			dist = getDistance()
			time.sleep(1)
				# if dist is not None and dist < distanceThreshold:
			rotate360()
			time.sleep(1)
			time.sleep(.5)
			stop()
			medicineIndex += 1
			lcd.clear()
			itsTime = False  # reset

		if medicineIndex == len(arrSched):
			lcdWrite("all schedule finished")
			medicineIndex = 0
			
except KeyboardInterrupt:
	pwm.stop()
	GPIO.cleanup()
