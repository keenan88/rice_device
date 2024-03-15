from gpiozero import DigitalOutputDevice
from time import sleep
import RPi.GPIO as GPIO

usleep = lambda x: sleep(x/1000000.0)

# right step: 23
# right dir: 24

# step left: 14
# left direction: 15

right_step_pin = DigitalOutputDevice(pin = 23, active_high=True, initial_value=False)
right_dir_pin = DigitalOutputDevice(pin = 24, active_high=True, initial_value=False)

left_step_pin = DigitalOutputDevice(pin = 14, active_high=True, initial_value=False)
left_dir_pin = DigitalOutputDevice(pin = 15, active_high=True, initial_value=False)



right_dir_pin.off()
left_dir_pin.off()

right_step_pin.off() 
left_step_pin.off() 

sleep(1)

while True:

    right_step_pin.on() 
    usleep(20)
    right_step_pin.off() 
    usleep(20) # Minimum 1.9us low time, as per page 8 of drv8834 datasheet

    left_step_pin.on() 
    usleep(20)
    left_step_pin.off() 
    usleep(20) # Minimum 1.9us low time, as per page 8 of drv8834 datasheet

    sleep(0.01)

