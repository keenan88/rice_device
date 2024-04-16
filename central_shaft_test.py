import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO
from gpiozero import Button
from time import sleep

in1 = 10
in2 = 9
laPin = 4
buttonPin = 11
upperbound = 17600
lowerbound = 500

# Linear actuator setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(laPin,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
la=GPIO.PWM(laPin,1000)

la.start(0)

# Pot/Button setup
but = Button(buttonPin, pull_up=False)
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
pot = AnalogIn(ads, ADS.P0)

print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

# while True: # Run forever
#     print(pot.value)
#     if but.is_pressed:
#         print("Button was pushed!")
# goingDown = true

import sys
import select
import tty
import termios

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)
try:
    tty.setcbreak(sys.stdin.fileno())

    while 1:
        if but.is_pressed:
            print("Button was pushed!")
        # print(pot.value)
        if isData():
            c = sys.stdin.read(1)
            # print(c)
            if c == '\x1b':         # x1b is ESC
                GPIO.output(in1,GPIO.LOW)
                GPIO.output(in2,GPIO.LOW)
                la.ChangeDutyCycle(0)
                break
            elif c == 'w' and pot.value > lowerbound:
                # print('goingup')
                GPIO.output(in1,GPIO.LOW)
                GPIO.output(in2,GPIO.HIGH)
            elif c == 's' and pot.value < upperbound:
                GPIO.output(in1,GPIO.HIGH)
                GPIO.output(in2,GPIO.LOW)
            elif c == 'l':
                la.ChangeDutyCycle(25)
            else:
                GPIO.output(in1,GPIO.LOW)
                GPIO.output(in2,GPIO.LOW)

        if but.is_pressed or pot.value > upperbound or pot.value < lowerbound:
            # print('limit reached', upperbound, lowerbound, but.is_pressed)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.LOW)


finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# while(1):

#     x=input("gimme seom text")


#     if x=='f':
#         print("forward")
#         GPIO.output(in1,GPIO.HIGH)
#         GPIO.output(in2,GPIO.LOW)

#     elif x=='b':
#         print("backward")
#         GPIO.output(in1,GPIO.LOW)
#         GPIO.output(in2,GPIO.HIGH)

#     elif x=='l':
#         print("low")
#         la.ChangeDutyCycle(25)
#     elif x=='m':
#         print("medium")
#         la.ChangeDutyCycle(50)

#     elif x=='h':
#         print("high")
#         la.ChangeDutyCycle(75)
     
#     elif x=='p':
#         print('Pot: ' + str(pot.value))
        

#     elif x=='e':
#         la.stop()
#         GPIO.cleanup()
#         break
    
#     else:
#         GPIO.output(in1,GPIO.LOW)
#         GPIO.output(in2,GPIO.LOW)