import binascii
import serial
import time
import RPi.GPIO as GPIO
import qwiic_vl53l1x
import qwiic_tca9548a
import cv2

sysRunning_flag = True

#LESSON: MUST PUT INTO SAFE MODE BEFORE STOP COMMAND 
ser=serial.Serial(port='/dev/ttyUSB0',baudrate=115200)
ser.flushOutput()

# Arm setup
arm=serial.Serial(port='/dev/ttyS0',baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)

# # TOF sensor setup
# mux = qwiic_tca9548a.QwiicTCA9548A()
# mux.disable_all()
# mux.enable_channels(0)
# ToF = qwiic_vl53l1x.QwiicVL53L1X()
# ToF.sensor_init()
# if (ToF.sensor_init() == None):                  # Begin returns 0 on a good init
    # print("Sensor front online!\n")
# ToF.set_distance_mode(2)
# mux.disable_all()
# mux.enable_channels(6)
# ToF.sensor_init()
# if (ToF.sensor_init() == None):                  # Begin returns 0 on a good init
    # print("Sensor 6 right online!\n")
# ToF.set_distance_mode(2)
# mux.disable_all()
# mux.enable_channels(7)
# ToF.sensor_init()
# if (ToF.sensor_init() == None):                  # Begin returns 0 on a good init
    # print("Sensor 7 left online!\n")
# ToF.set_distance_mode(2)

# Initialize camera
FOCAL = 220 # camera focal length
LENGTH_MOUSE = 14.3
# dist_calc = WIDTH_BALL * FOCAL / dist_rad

classNames= []
classFile = 'coco.names'
with open(classFile,'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

configPath = 'graph.pbtxt'
weightsPath = 'frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

def stop():
	ser.write(b'\x92\x00\x00\x00\x00')
	time.sleep(.1)
  
def rotate90(clockwise): # rotate 90 deg
	resetAngle()
	angleRotated = 0 # variable to calculated amount of rotation
	if clockwise == 0: # ccw
		print("Detected, rotate ccw")
		ser.write(b'\x92\x00\x1E\xFF\xE2') # rotate counterclockwise
		while angleRotated < 90:
			# Send commmand to get angle rotation
			ser.write(b'\x8E\x14') # number 20 = \x14 angle
			time.sleep(.2)
			while ser.inWaiting() != 0: # receive message from iRobot, instead of polling maybe use interrupts/callbacks
				response = []
				for i in range(2):
					response.append(hex(ord(ser.read())))
				angleRotated = angleRotated + int(response[1], 16)
				print(angleRotated)
	else: # cw
		print("Detected, rotate cw")
		ser.write(b'\x92\xFF\xE2\x00\x1E') # rotate clockwise
		while angleRotated < 90:
			# Send commmand to get angle rotation
			ser.write(b'\x8E\x14') # number 20 = \x14 angle
			time.sleep(.2)
			while ser.inWaiting() != 0: # receive message from iRobot, instead of polling maybe use interrupts/callbacks
				response = []
				for i in range(2):
					temp = ser.read()
					#print(temp)
					response.append(hex(ord(temp)))
				temp = int(response[1], 16)
				temp = temp ^ 0b11111111
				if temp != 255:				
					angleRotated = angleRotated + temp + 1
				print(angleRotated)
	stop()
	
def rotate5(clockwise): # rotate 90 deg
	resetAngle()
	angleRotated = 0 # variable to calculated amount of rotation
	if clockwise == 0: # ccw
		print("Detected, rotate ccw")
		ser.write(b'\x92\x00\x1E\xFF\xE2') # rotate counterclockwise
		while angleRotated < 5:
			# Send commmand to get angle rotation
			ser.write(b'\x8E\x14') # number 20 = \x14 angle
			time.sleep(.2)
			while ser.inWaiting() != 0: # receive message from iRobot, instead of polling maybe use interrupts/callbacks
				response = []
				for i in range(2):
					response.append(hex(ord(ser.read())))
				angleRotated = angleRotated + int(response[1], 16)
				print(angleRotated)
	else: # cw
		print("Detected, rotate cw")
		ser.write(b'\x92\xFF\xE2\x00\x1E') # rotate clockwise
		while angleRotated < 5:
			# Send commmand to get angle rotation
			ser.write(b'\x8E\x14') # number 20 = \x14 angle
			time.sleep(.2)
			while ser.inWaiting() != 0: # receive message from iRobot, instead of polling maybe use interrupts/callbacks
				response = []
				for i in range(2):
					temp = ser.read()
					#print(temp)
					response.append(hex(ord(temp)))
				temp = int(response[1], 16)
				temp = temp ^ 0b11111111
				if temp != 255:				
					angleRotated = angleRotated + temp + 1
				print(angleRotated)
	stop()

	
def forwardShort():
	global stopDisinfection
	startTime = time.time()
	ser.write(b'\x92\x00\x2f\x00\x2f') #wheel speed of 8f
        #print("move forward for some time")
	
	move = 1 # Variable on whether we should keep moving forward
	while (move):
		if (getBumpSensor() == 1 or getProxSensorAll() == 1): # If detect a table leg, we should stop disinfecting
		    stop()
		    move = 0
		    stopDisinfection = True
		    
		elif (time.time() - startTime > 2.5): # If more than 2 sec
			stop()
			move = 0
			
def backwardShort():
	startTime = time.time()
	ser.write(b'\x92\xFF\xD8\xFF\xD8') # Move wheels backward
	
	move = 1 # Variable on whether we should keep moving forward
	while (move):
		if (time.time() - startTime > 1.5): # If more than 1.5 sec
			stop()
			move = 0

def forwardUntilSensor(): # move forward until outside table based on ToF
	global stopDisinfection
	print("Move forward until outside table based on ToF")
	
	ser.write(b'\x92\x00\x2f\x00\x2f') #wheel speed of 8f

	startTime2 = time.time() # Stopwatch to measure time since function first started
	
	move = 1 # Variable on whether we should keep moving forward
	underTable = 1 # Variable on whether robot is under a table or not
	for i in range(20): # throw first measurements
		getFrontTof()
	while (move):
		if (getBumpSensor() == 1 or getProxSensorAll() == 1): # If detect a table leg, we should stop disinfecting
			stop()
			move = 0
			stopDisinfection = True

		if (underTable == 1): # Table detected
			if getFrontTof() == 0: # If table no longer detected
				if (time.time() - startTime2 < 1): # If robot quit table before 1 second, we should stop disinfecting
					stop()
					move = 0
					stopDisinfection = True
				underTable = 0
				startTime = time.time() # Store time since table first not detected

		else: # No table detected
			if (time.time() - startTime > 2): # Has been out of table for 2sec
				stop()
				move = 0

def lookForLeg(): # Keep moving forward, and stop when leg is detected
	ser.write(b'\x92\x00\x2f\x00\x2f') #wheel speed of 2f
	print("Looking for leg")
	move = 1 # Variable on whether we should keep moving forward
	while (move):
		if (getBumpSensor() == 1): # If leg is detected, stop
			stop()
			move = 0
		
		# Use ToF to maintain moving along table. RightToF = 1 and Left ToF = 0
		if (getRightTof() == 0): # If right ToF no longer detects table, slow down right wheel to steer right
			ser.write(b'\x92\x00\x20\x00\x2f')
		elif (getLeftTof() == 1): # If left ToF detects table, slow down left wheel to steer left
			ser.write(b'\x92\x00\x2f\x00\x20')
		else:
			ser.write(b'\x92\x00\x2f\x00\x2f')

def alignToTable(): # Align robot to table when facing to it using ToF sensor
	print("Align to table")
	for i in range(20):
		getRightTof()
		getLeftTof()
	move = 1 # Variable on whether we should keep moving forward
	while (move):
		if (getRightTof() == 1 and getLeftTof() == 1): # If both sensor detects table, stop as table is aligned
			stop()
			move = 0
		elif (getRightTof() == 1 and getLeftTof() == 0): # If right ToF detects table, move only left wheel
			ser.write(b'\x92\x00\x00\x00\x2f')
		elif (getRightTof() == 0 and getLeftTof() == 1): # If left ToF detects table, move only right wheel
			ser.write(b'\x92\x00\x2f\x00\x00')
		else: # no ToF sensor detects table yet
			ser.write(b'\x92\x00\x2f\x00\x2f')


def getFrontTof(): # Measure using forward ToF sensor, return 1 if Roomba is under table
	mux.disable_all()
	mux.enable_channels(0)
	ToF.start_ranging() # Write configuration bytes to initiate measurement
	time.sleep(.005)
	distance = ToF.get_distance() # Get the result of the measurement from the sensor
	time.sleep(.005)
	ToF.stop_ranging()
        #print(distance)
	if (distance < 1300): # may need to be adjusted
		return 1
	else:
		return 0

def getRightTof(): # Measure using right ToF sensor, return 1 if Roomba is under table
	mux.disable_all()
	mux.enable_channels(7)
	ToF.start_ranging() # Write configuration bytes to initiate measurement
	time.sleep(.005)
	distance = ToF.get_distance() # Get the result of the measurement from the sensor
	time.sleep(.005)
	ToF.stop_ranging()
        #print(distance)
	if (distance < 1300): # may need to be adjusted
		return 1
	else:
		return 0

def getLeftTof(): # Measure using left ToF sensor, return 1 if Roomba is under table
	mux.disable_all()
	mux.enable_channels(6)
	ToF.start_ranging() # Write configuration bytes to initiate measurement
	time.sleep(.005)
	distance = ToF.get_distance() # Get the result of the measurement from the sensor
	time.sleep(.005)
	ToF.stop_ranging()
        #print(distance)
	if (distance < 1300): # may need to be adjusted
		return 1
	else:
		return 0
		    
def getProxSensorAll(): # Get all prox sensor reading, return 1 if object detected
	ser.write(b'\x8E\x2D') #45 = 2D all, 48 = \x30 center left, 49 = \x31 center right
	time.sleep(.2)
	while ser.inWaiting() != 0:
		response = []
		response.append(hex(ord(ser.read())))
		if int(response[0], 16) > 0 : # If object detected form one of the sensor
			return 1
		else:
			return 0

def getBumpSensor(): # Get bump and wheeldrop sensor, return 1 if something detected
	ser.write(b'\x8E\x07') 
	time.sleep(.2)
	while ser.inWaiting() != 0:
		response = []
		response.append(hex(ord(ser.read())))
		if int(response[0], 16) > 0 : # If bump detected or robot lifted
			return 1
		else:
			return 0
		    
def resetAngle(): # Reset angle calculation
	ser.write(b'\x8E\x14')
	time.sleep(.2)
	while ser.inWaiting() != 0:
		ser.read()
		
def resetDistance(): # Reset distance calculation
	ser.write(b'\x8E\x13')
	time.sleep(.2)
	while ser.inWaiting() != 0:
		ser.read()

# Set arm positions
def armUp():
	arm.write(b'\x55\x55\x05\x06\x01\x01\x00') # update position to ID number
    
def armDown():
	arm.write(b'\x55\x55\x05\x06\x02\x01\x00')
    
def armClean():
	arm.write(b'\x55\x55\x05\x06\x03\x01\x00')

def armRight():
	arm.write(b'\x55\x55\x05\x06\position\x01\x00')

def armStop():
	arm.write(b'\x55\x55\x02\x07')
	
def getObjects(frame,thres,nms,draw=True,objects=[]):
    classIds, confs, bbox = net.detect(frame,confThreshold=thres,nmsThreshold=nms)
    robotMode = 1
    if len(objects) == 0: 
        objects = classNames
    objectInfo = []
    
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId - 1]
            
            if className in objects:
                objectInfo.append([box, className])
                
                if (draw):
                    
                    #cv2.rectangle(frame,box,color=(0,0,255))
                    #cv2.putText(frame,className.upper(),(box[0]+10,box[1]+30),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                    #cv2.putText(frame,str(round(confidence*100,2)),(box[0]+100,box[1]+30), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                    
                    
                    x_mid = box[0] + box[2]/2 # Calculated and verified to be x midpoint
                    object_ratio = (box[2]*box[3])/(320*320)
                    # If x_mid < 100, turn one dir, if x_mid > 220, turn another dir
                    if x_mid < 100:
                        robotMode = 4
				
			
                    elif x_mid > 220:
                        robotMode = 3

                    
                    
                    # if less than 0.4, move forward, else lean arm forward
                    elif object_ratio < 0.3:
                        robotMode = 2
                    else:
                        robotMode = 5
                    
    return frame,objectInfo,robotMode

GPIO.setmode(GPIO.BCM)   #set up GPIO pins
# GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#GPIO.setup(trigPin, GPIO.OUT, initial = GPIO.LOW)
#GPIO.setup(echoPin, GPIO.IN)

time.sleep(1)

## add callback event
# GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)

time.sleep(0.2)
ser.write(b'\x80') #start
print("started")
time.sleep(0.2)
ser.write(b'\x83')#safe mode
time.sleep(0.2)

cap = cv2.VideoCapture(-1)
cap.set(3, 320)
cap.set(4, 320)

# robotMode: 1 == wander and look for table, 2 == initial setup when detecting a table, 3 == disinfect table
robotMode = 1 # If True, robot will wander in cleanmode to look for table. if false will disinfect table

stopDisinfection = False # set to True if entire table has been disinfected, verified by bump and prox sensors, or if error is predicted
lastKeyboardDisinfectTime = -10 # Set to -10s ago

try:
	while (sysRunning_flag):
		# View camera
		_, frame = cap.read()
		result,objectInfo,robotMode = getObjects(frame,0.5,0.2,objects=["keyboard"])
		#cv2.imshow('Output', frame)
		#print(found)
		cv2.waitKey(1)
		
		# No keyboard found
		if robotMode == 1:
			armDown()
			print("forward")
			#ser.write(b'\x92\x00\x2f\x00\x2f') #wheel speed of 2f
		
		# Keyboard found, keep moving forward
		elif robotMode == 2:
			armDown()
			print("forward found")
			#ser.write(b'\x92\x00\x20\x00\x20') #wheel speed of 20
			
		# Steer right
		elif robotMode == 3:
			armDown()
			#rotate5(1)
			print("rotate cw")
			#ser.write(b'\x92\x00\x1E\xFF\xE2') # rotate counterclockwise
			#time.sleep(2)
			
		# Steer left
		elif robotMode == 4:
			armDown()
			#rotate5(0)
			print("rotate ccw")
			#ser.write(b'\x92\xFF\xE2\x00\x1E') # rotate clockwise
			#time.sleep(2)
			
		# Disinfect
		elif robotMode == 5 and time.time() - lastKeyboardDisinfectTime > 5:
			stop()
			time.sleep(0.5)
			print("cleaning with arm")
			armClean()
			time.sleep(6)
			lastKeyboardDisinfectTime = time.time()

except KeyboardInterrupt:
	stop()
	armStop()
	GPIO.cleanup() # clean up GPIO on CTRL+C exit
    	#safe mode then stop
	ser.write(b'\x83')
	time.sleep(0.2)
	#stop command when we are done working
	ser.write(b'\xAD')
	ser.close()

    
print("exit")
#safe mode then stop
time.sleep(0.2)
ser.write(b'\x83')#safe mode, must be in safe mode before stopping
time.sleep(0.2)
ser.write(b'\x92\x00\x00\00\00') #wheel speed of 0
time.sleep(0.2)
#stop command when we are done working
ser.write(b'\xAD') #stop
ser.close()
GPIO.cleanup()
