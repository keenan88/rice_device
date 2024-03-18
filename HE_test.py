#from gpiozero import DigitalInputDevice
from time import sleep
import RPi.GPIO as GPIO

usleep = lambda x: sleep(x/1000000.0)

pin_to_test = 25

GPIO.setmode(GPIO.BCM)

GPIO.setup(pin_to_test, GPIO.IN)
#GPIO.setup(pin_to_test, GPIO.OUT)
#GPIO.output(pin_to_test, GPIO.LOW)

while 1:
    print(GPIO.input(pin_to_test))
    sleep(0.1)

"""
GPIO: Flopper position
1's are at the front of the robot, 4's are at the back
6 - L3
13 - L4
19 - L1
26 - L2

21 - C3
20 - C1
16 - C2
12 - C4

7 - R2
8 - R1
25 - R3

"""