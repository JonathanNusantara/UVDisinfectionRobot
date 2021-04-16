import serial
import time
import RPi.GPIO as GPIO

sysRunning_flag = True

trigPin = 20 # front: 20, left: 5, right: 19
echoPin = 21 # front: 21 left: 6, right: 26

frontTrigPin = 20
frontEchoPin = 21

leftTrigPin = 5
leftEchoPin = 6

rightTrigPin = 19
rightEchoPin = 26

def checklistFront():
    GPIO.output(frontTrigPin, GPIO.HIGH)
    time.sleep(0.00015)
    GPIO.output(frontTrigPin, GPIO.LOW)

    t1 = time.time()
    
    while not GPIO.input(frontEchoPin):
        pass
#    t1 = time.time()
    
    while GPIO.input(frontEchoPin):
        pass
    t2 = time.time()
    
    return (t2 - t1) * 340 * 100 / 2

def checklistLeft():
    GPIO.output(leftTrigPin, GPIO.HIGH)
    time.sleep(0.00015)
    GPIO.output(leftTrigPin, GPIO.LOW)

    t1 = time.time()
    
    while not GPIO.input(leftEchoPin):
        pass
#    t1 = time.time()
    
    while GPIO.input(leftEchoPin):
        pass
    t2 = time.time()
    
    return (t2 - t1) * 340 * 100 / 2
    
def checklistRight():
    GPIO.output(rightTrigPin, GPIO.HIGH)
    time.sleep(0.00015)
    GPIO.output(rightTrigPin, GPIO.LOW)

    t1 = time.time()
    
    while not GPIO.input(rightEchoPin):
        pass
#    t1 = time.time()
    
    while GPIO.input(rightEchoPin):
        pass
    t2 = time.time()
    
    return (t2 - t1) * 340 * 100 / 2
        
def GPIO27_callback(channel):
    print ("")
    print "Button 27 pressed..."
    global sysRunning_flag
    sysRunning_flag = False
    print("System shut down")
    

GPIO.setmode(GPIO.BCM)   #set up GPIO pins
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

## GPIO setup for ultrasound's pins
GPIO.setup(frontTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(frontEchoPin, GPIO.IN)

GPIO.setup(leftTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(leftEchoPin, GPIO.IN)

GPIO.setup(rightTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(rightEchoPin, GPIO.IN)

time.sleep(1)

## add callback event
GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)

try:
    while (sysRunning_flag):
        time.sleep(0.5)
        print('calculating distance....')
        dist = checklistFront()
        print('Distance front: %0.2f cm'%dist)
        time.sleep(0.5)
        dist = checklistLeft()
        print('Distance left: %0.2f cm'%dist)
        time.sleep(0.5)
        dist = checklistRight()
        print('Distance right: %0.2f cm'%dist)
        

except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    
print("exit")
GPIO.cleanup()
