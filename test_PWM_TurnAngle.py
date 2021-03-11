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
  
def rotate_robot():
	print("Detected, rotate 90 deg")
	ser.write('\x83')#safe mode
	time.sleep(0.2)
	ser.write('\x92\x00\x5F\xFF\xA1') # rotate clockwise
	time.sleep(4)
	
	angleRotated = 0 # variable to calculated amount of rotation
	
#	while angleRotated < 90:
#		# Send commmand to get angle rotation
#		ser.write('\x8E\x14') # number 20 = \x14 angle
#       	time.sleep(.2)
#		while ser.inWaiting() != 0: # receive message from iRobot, instead of polling maybe use interrupts/callbacks
#            		response = []
#            		for i in range(2):
#                		response.append(hex(ord(ser.read())))
#            		print(response)
#			print(int(response[1], 16))
#			angleRotated = angleRotated + int(response[1], 16)
#			print(angleRotated)
#			print("done small movement")
		
	
	
	
def forward():
	ser.write('\x92\x00\x8f\00\8f') #wheel speed of 8f
        print("continue moving forward")
	
	distanceMoved = 0 # variable to calculated amount of rotation
	
#	while distanceMoved < 500:# 500mm = 50cm
#		# Send commmand to get angle rotation
#		ser.write('\x8E\x13') # number 19 = \x13 angle
#        	time.sleep(.2)
#		while ser.inWaiting() != 0: # receive message from iRobot, instead of polling maybe use interrupts/callbacks
#            		response = []
#            		for i in range(2):
#                		response.append(hex(ord(ser.read())))
#            		print(response)
#			print(int(response[1], 16))
#			distanceMoved = distanceMoved + int(response[1], 16)
#			print(distanceMoved)
#			print("done small movement")

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
ser.write('\x83')#safe mode
time.sleep(0.2)
ser.write('\x92\x00\x8f\00\8f') #both wheel speed of 8f
time.sleep(0.2)
print("start moving")

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
                rotate_robot()
                time.sleep(.2)
            forward()
         

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
