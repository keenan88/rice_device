import RPi.GPIO as GPIO
from gpiozero import Button, InputDevice
from time import sleep

in1 = 10
in2 = 9
laPin = 2
buttonPin = 11
potPin = 4

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(laPin,GPIO.OUT)

but = Button(buttonPin, pull_up=False)
pot = InputDevice(potPin)
# GPIO.setup(button, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# GPIO.setup(pot, GPIO.IN)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
la=GPIO.PWM(laPin,1000)

la.start(0)

print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

# while True: # Run forever
#     print(pot.value)
#     if but.is_pressed:
#         print("Button was pushed!")
    
        # print("not pushed")
while(1):

    x=input("gimme seom text")


    if x=='f':
        print("forward")
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)

    elif x=='b':
        print("backward")
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)

    elif x=='l':
        print("low")
        la.ChangeDutyCycle(25)
    elif x=='m':
        print("medium")
        la.ChangeDutyCycle(50)

    elif x=='h':
        print("high")
        la.ChangeDutyCycle(75)
     
    elif x=='p':
        print('Pot: ' + str(pot.value))
        

    elif x=='e':
        la.stop()
        c.stop()
        l.stop()
        GPIO.cleanup()
        break
    
    else:
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)