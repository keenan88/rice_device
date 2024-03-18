#from gpiozero import DigitalInputDevice
from time import sleep
import RPi.GPIO as GPIO

usleep = lambda x: sleep(x/1000000.0)

# Each sub vector is ordered L, C, R
# First vector is first row, second vector is second row, etc.
pin_numbers = [
    [19, 20, 8],
    [26, 16, 7],
    [6, 21, 25],
    [13, 12, 23]
]

GPIO.setmode(GPIO.BCM)

for flopper_row in pin_numbers:

    for pin_num in flopper_row:
        # Ensure all pins have no residual voltage on their lines.
        GPIO.setup(pin_num, GPIO.OUT)
        GPIO.output(pin_num, GPIO.LOW)

        # Setup HE pin to read
        GPIO.setup(pin_num, GPIO.IN)
        
        print(pin_num)

    print()


input()


while 1:
    
    for flopper_row in pin_numbers:

        for pin_num in flopper_row:

            print(GPIO.input(pin_num), end=', ')

        print()

    print()
    print()
    sleep(0.1)

"""
GPIO: Flopper position
1's are at the front of the robot, 4's are at the back
19 - L1
26 - L2
6 - L3
13 - L4

20 - C1
16 - C2
21 - C3
12 - C4

8 - R1
7 - R2
25 - R3
23 - R4

"""