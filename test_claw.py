import RPi.GPIO as GPIO
import time
import atexit

atexit.register(GPIO.cleanup)

servopin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(servopin, GPIO.OUT, initial=False)

p = GPIO.PWM(servopin, 50)

angle = 60
duty = 2.5 + 10 * angle / 180  # duty cycle for 60°
p.start(duty)  # move to position
time.sleep(1)  # allow servo to reach position
p.ChangeDutyCycle(0)  # stop sending signal to reduce jitter

try:
    while True:
        time.sleep(1)  # do nothing, just maintain program

except KeyboardInterrupt:
    print("Stopping...")
    p.stop()
    GPIO.cleanup()