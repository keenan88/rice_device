import RPi.GPIO as GPIO
from time import sleep

in1 = 0
in2 = 5
right = 22
center = 27
left = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(right,GPIO.OUT)
GPIO.setup(center,GPIO.OUT)
GPIO.setup(left,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
r=GPIO.PWM(right,1000)
c=GPIO.PWM(center,1000)
l=GPIO.PWM(left,1000)

r.start(0)
c.start(0)
l.start(0)

print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

while(1):

    x=input("gimme seom text")
    
    # if x=='r':
    #     print("run")
    #     if(temp1==1):
    #      GPIO.output(in1,GPIO.HIGH)
    #      GPIO.output(in2,GPIO.LOW)
    #      print("forward")
    #      x='z'
    #     else:
    #      GPIO.output(in1,GPIO.LOW)
    #      GPIO.output(in2,GPIO.HIGH)
    #      print("backward")
    #      x='z'


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
        r.ChangeDutyCycle(25)
        c.ChangeDutyCycle(25)
        l.ChangeDutyCycle(25)

    elif x=='m':
        print("medium")
        r.ChangeDutyCycle(50)
        c.ChangeDutyCycle(50)
        l.ChangeDutyCycle(50)

    elif x=='h':
        print("high")
        r.ChangeDutyCycle(100)
        c.ChangeDutyCycle(100)
        l.ChangeDutyCycle(100)
     
    
    elif x=='e':
        r.stop()
        c.stop()
        l.stop()
        GPIO.cleanup()
        break
    
    else:
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)