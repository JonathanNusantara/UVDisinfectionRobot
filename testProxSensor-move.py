import serial
import time
import RPi.GPIO as GPIO

sysRunning_flag = True

#LESSON: MUST PUT INTO SAFE MODE BEFORE STOP COMMAND 
ser=serial.Serial(port='/dev/ttyUSB0',baudrate=115200)
ser.flushOutput()


#---------polling--------
trigPin = 20
echoPin = 21

def check_ultrasonic():
    GPIO.output(trigPin, GPIO.HIGH)
    time.sleep(0.00015)
    GPIO.output(trigPin, GPIO.LOW)

    while not GPIO.input(echoPin):
        pass
    t1 = time.time()
    
    while GPIO.input(echoPin):
        pass
    t2 = time.time()
    
    return (t2 - t1) * 340 * 100 / 2

# def GPIO27_callback(channel):
    # print ("")
    # print "Button 27 pressed..."
    # global sysRunning_flag
    # sysRunning_flag = False
    # print("System shut down")

def fakeLawnMow():
	print("Fake lawn mow is just moving forward slowly for 4 seconds")
	ser.write('\x83')#safe mode
	time.sleep(0.2)
	ser.write('\x92\x00\x33\x00\x33')
	time.sleep(4)

def stop_robot():
	print("Table leg detected, stop robot")
	ser.write('\x83')#safe mode
	time.sleep(0.2)
	ser.write('\x92\x00\x00\x00\x00')
	time.sleep(.2)

GPIO.setmode(GPIO.BCM)   #set up GPIO pins
# GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(trigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(echoPin, GPIO.IN)

time.sleep(1)

## add callback event
# GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)

#----------start in random walk----------
time.sleep(0.2)
ser.write('\x80') #start
print("started")

time.sleep(0.2)
ser.write('\x87') #clean mode
print("clean")
# ser.write('\x83')#safe mode
# time.sleep(0.2)
# ser.write('\x92\x00\x00\00\00') #wheel speed of 0
# time.sleep(0.2)
# print("start moving")

detected = False # Variable for object detected status
moving = True # Variable for moving status

try:
    while (sysRunning_flag):
        time.sleep(0.1)
        ser.write('\x8E\x30') #45 = 2D all, 48 = \x30 center left, 49 = \x31 center right
        time.sleep(.2)
        while ser.inWaiting() != 0: # receive message from iRobot, instead of polling maybe use interrupts/callbacks
            response = []
            for i in range(2):
                response.append(hex(ord(ser.read())))
            print(response)
            if (int(response[0], 16) > 0 or int(response[1], 16) > 20): # If close, accurate distance not defined
                if moving == True:
                    stop_robot()
                    moving = False
                    detected = True
                    time.sleep(.2)
            else: # nothing detected
                if detected == True:
                    print("no longer detected")
                    detected = False    
        if detected == False and moving == False : # If no object detected and is not currently moving
            # ser.write('\x87') #clean mode
            ser.write('\x87') #clean mode
            print("continue cleaning")
            moving = True

        
        
        
        # print('calculating distance....')
        # dist = check_ultrasonic()
        # print('Distance: %0.2f cm'%dist)
        # #--------move to some lawn mowing state----------
        # if dist < 30:
            # fakeLawnMow()
            # #---------return to polling in wander mode----------
            # time.sleep(0.2)

            # ser.write('\x80') #start
            # print("started")

            # time.sleep(0.2)
            # ser.write('\x87') #clean mode
            # print("clean")
            # time.sleep(4)
            # break

         

except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    #safe mode then stop
    ser.write('\x83')
    time.sleep(0.2)
    #stop command when we are done working
    ser.write('\xAD')
    ser.close()

    
print("exit")
#safe mode then stop
time.sleep(0.2)
ser.write('\x83')#safe mode, must be in safe mode before stopping
time.sleep(0.2)
ser.write('\x92\x00\x00\00\00') #wheel speed of 0
time.sleep(0.2)
#stop command when we are done working
ser.write('\xAD') #stop
ser.close()
GPIO.cleanup()


