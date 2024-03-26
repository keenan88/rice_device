from gpiozero import DigitalOutputDevice
from time import sleep
import RPi.GPIO as GPIO

usleep = lambda x: sleep(x/1000000.0)

GPIO.setmode(GPIO.BCM)

right_step_pin = 14
right_dir_pin = 24

left_step_pin = 15
left_dir_pin = 18

GPIO.setup(left_step_pin, GPIO.OUT)
GPIO.setup(left_dir_pin, GPIO.OUT)

GPIO.setup(right_step_pin, GPIO.OUT)
GPIO.setup(right_dir_pin, GPIO.OUT)

GPIO.output(left_dir_pin, GPIO.HIGH)
GPIO.output(right_dir_pin, GPIO.LOW)

sleep(1)

while 1:

    GPIO.output(right_step_pin, GPIO.LOW)
    usleep(1000)
    GPIO.output(right_step_pin, GPIO.HIGH)
    usleep(1000)

    GPIO.output(left_step_pin, GPIO.LOW)
    usleep(1000)
    GPIO.output(left_step_pin, GPIO.HIGH)
    usleep(1000)

    sleep(0.5)

