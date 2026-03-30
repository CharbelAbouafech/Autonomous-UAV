
import RPi.GPIO as GPIO
import time 
import signal
import atexit

atexit.register(GPIO.cleanup)

servopin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(servopin, GPIO.OUT)
p = GPIO.PWM(servopin, 50)

p.start(0)
time.sleep(2)


open_angle = 19
close_angle = 190.98

def set_angle(angle):
	#print(f"Testing {angle} , duty cycle {duty}")
	#duty = 3 + 9 * angle /180
	duty = 2.5 + (10 * angle /180)
	p.ChangeDutyCycle(duty)
	time.sleep(0.5)
	p.ChangeDutyCycle(0)

	
	
while(True):
	user = input ("Enter command: (o = open, c = close, e = exit)").strip().lower()
	if user == 'o':
		set_angle(open_angle) 
		print("Opening Claw")
	elif user == 'c':
		print("Closing Claw")
		set_angle(close_angle)
	elif user == 'e':
		print("Exiting")
		break
	else:
		print("Invalid Comment")