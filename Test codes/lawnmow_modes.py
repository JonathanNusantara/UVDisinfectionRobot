import binascii
import serial
import time
import RPi.GPIO as GPIO
import qwiic_vl53l1x

sysRunning_flag = True

#LESSON: MUST PUT INTO SAFE MODE BEFORE STOP COMMAND 
ser=serial.Serial(port='/dev/ttyUSB0',baudrate=115200)
ser.flushOutput()

# TOF sensor setup
ToF = qwiic_vl53l1x.QwiicVL53L1X()
if (ToF.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor online!\n")
ToF.set_distance_mode(2)

def stop():
	ser.write('\x92\x00\x00\x00\x00')
	time.sleep(.1)
  
def rotate90(clockwise): # rotate 90 deg
	resetAngle()
	angleRotated = 0 # variable to calculated amount of rotation
	if clockwise == 0: # ccw
		print("Detected, rotate ccw")
		ser.write('\x92\x00\x1E\xFF\xE2') # rotate counterclockwise
		while angleRotated < 90:
			# Send commmand to get angle rotation
			ser.write('\x8E\x14') # number 20 = \x14 angle
			time.sleep(.2)
			while ser.inWaiting() != 0: # receive message from iRobot, instead of polling maybe use interrupts/callbacks
				response = []
				for i in range(2):
					response.append(hex(ord(ser.read())))
				#print(response)
				#print(response)
				angleRotated = angleRotated + int(response[1], 16)
				print(angleRotated)
	else: # cw
		print("Detected, rotate cw")
		ser.write('\x92\xFF\xE2\x00\x1E') # rotate clockwise
		while angleRotated < 90:
			# Send commmand to get angle rotation
			ser.write('\x8E\x14') # number 20 = \x14 angle
			time.sleep(.2)
			while ser.inWaiting() != 0: # receive message from iRobot, instead of polling maybe use interrupts/callbacks
				response = []
				for i in range(2):
					temp = ser.read()
					#print(temp)
					response.append(hex(ord(temp)))
				#print(response[1])
				#print(format(int(response[1],16)))
				#temp = ~response[1]
				#print(temp)
				temp = int(response[1], 16)
				#print(type(temp))
				temp = temp ^ 0b11111111
				#print(temp)
				#print(type(temp))
				#temp = ~(0xffff - temp) + 1
				#print(bin(temp) ^ 0b11111111)
				if temp != 255:				
					angleRotated = angleRotated + temp + 1
				print(angleRotated)
	stop()
		
	
	
	
def forwardShort():
	global stopDisinfection
	startTime = time.time()
	ser.write('\x92\x00\x2f\x00\x2f') #wheel speed of 8f
        #print("move forward for some time")
	
	move = 1 # Variable on whether we should keep moving forward
	while (move):
		if (getBumpSensor() == 1 or getProxSensorAll() == 1): # If detect a table leg, we should stop disinfecting
		    stop()
		    move = 0
		    stopDisinfection = True
		    
		elif (getProxSensorMid() == 1 or time.time() - startTime > 2): # If object detected or more than 2 sec
			stop()
			move = 0
			
def backwardShort():
	startTime = time.time()
	ser.write('\x92\xFF\xE2\xFF\xE2') # Move wheels backward
	
	move = 1 # Variable on whether we should keep moving forward
	while (move):
		if (time.time() - startTime > 1.5): # If more than 1.5 sec
			stop()
			move = 0
	
	
	#distanceMoved = 0 # variable to calculated amount of rotation
#	while distanceMoved < 250:# 500mm = 50cm
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

def forwardUntilSensor(): # move forward until outside table based on ToF
	global stopDisinfection
	print("Move forward until outside table based on ToF")
	
	ser.write('\x92\x00\x2f\x00\x2f') #wheel speed of 8f
        #print("move forward for some time")
	
	move = 1 # Variable on whether we should keep moving forward
	underTable = 1 # Variable on whether robot is under a table or not
	for i in range(20): # throw first measurements
		getUnderTable()
	while (move):
		if (getBumpSensor() == 1 or getProxSensorAll() == 1): # If detect a table leg, we should stop disinfecting
		    stop()
		    move = 0
		    stopDisinfection = True
		    
		if underTable == 1: # Table detected
		    # Use prox sensor first, later change to ToF
		    if getUnderTable() == 0: # If table no longer detected
			underTable = 0
			startTime = time.time() # Store time since table first not detected
		else: # No table detected
		    if (time.time() - startTime > 2): # Has been out of table for 0.5sec
			stop()
			move = 0

def lookForLeg(): # Keep moving forward, and stop when leg is detected
	ser.write('\x92\x00\x2f\x00\x2f') #wheel speed of 2f
	print("Looking for leg")
	move = 1 # Variable on whether we should keep moving forward
	while (move):
		if (getBumpSensor() == 1):
			stop()
			move = 0
def getUnderTable(): # Measure using forward ToF sensor, return 1 if Roomba is under table
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

def getProxSensorMid(): # Get middle proximity sensor reading
	#print("Reading Prox sensor")
	ser.write('\x8E\x30') #45 = 2D all, 48 = \x30 center left, 49 = \x31 center right
        time.sleep(.2) # Can I shorten this?
	while ser.inWaiting() != 0: # receive message from iRobot, instead of polling maybe use interrupts/callbacks
            response = []
            for i in range(2):
                response.append(hex(ord(ser.read())))
	    if (int(response[0], 16) > 0 or int(response[1], 16) > 20): # Return 1 if there is a close object
		    return 1
	    else:
		    return 0
		    
def getProxSensorAll(): # Get all prox sensor reading, return 1 if object detected
	ser.write('\x8E\x2D') #45 = 2D all, 48 = \x30 center left, 49 = \x31 center right
	time.sleep(.2)
	while ser.inWaiting() != 0:
		response = []
		response.append(hex(ord(ser.read())))
		if int(response[0], 16) > 0 : # If object detected form one of the sensor
			return 1
		else:
			return 0

def getBumpSensor(): # Get bump and wheeldrop sensor, return 1 if something detected
	ser.write('\x8E\x07') 
	time.sleep(.2)
	while ser.inWaiting() != 0:
		response = []
		response.append(hex(ord(ser.read())))
		if int(response[0], 16) > 0 : # If bump detected or robot lifted
			return 1
		else:
			return 0
		    
def resetAngle(): # Reset angle calculation
	ser.write('\x8E\x14')
	time.sleep(.2)
	while ser.inWaiting() != 0:
		ser.read()
		
def resetDistance(): # Reset distance calculation
	ser.write('\x8E\x13')
	time.sleep(.2)
	while ser.inWaiting() != 0:
		ser.read()

GPIO.setmode(GPIO.BCM)   #set up GPIO pins
# GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#GPIO.setup(trigPin, GPIO.OUT, initial = GPIO.LOW)
#GPIO.setup(echoPin, GPIO.IN)

time.sleep(1)

## add callback event
# GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)

time.sleep(0.2)
ser.write('\x80') #start
print("started")
time.sleep(0.2)
ser.write('\x83')#safe mode
time.sleep(0.2)


#detected = False # Variable for object detected status
#moving = True # Variable for moving status

# robotMode: 1 == wander and look for table, 2 == initial setup when detecting a table, 3 == disinfect table
robotMode = 1 # If True, robot will wander in cleanmode to look for table. if false will disinfect table

stopDisinfection = False # set to True if entire table has been disinfected, verified by bump and prox sensors, or if error is predicted

try:
    while (sysRunning_flag):
	if robotMode == 1:
		# Set to clean mode
		ser.write('\x87')#safe mode
		time.sleep(0.2)
		for i in range(100):
			getUnderTable() # Throw away early measurements
		while (robotMode == 1): # Continue to look for table
			if (getUnderTable() == 1): # If table detected, go to robotMode 2
				robotMode = 2
	    
	elif robotMode == 2:
		# Set to safe mode
		ser.write('\x83')#safe mode
		time.sleep(0.2)
		
		# STEP 1: Align robot to table using ToF sensor
		backwardShort()
		# STEP 2: Rotate cccw
		print("initial ccw")
		rotate90(0) # Rotate ccw after aligning with table
		
		# STEP 3: Move along table while alinging using ToF sensor, will stop when table leg deteced
		print("move along table")
		lookForLeg() # Move along the table until table leg is detected
		backwardShort()
		# STEP 4: Rotate clockwise to face table again
		print("rotate cw towards table")
		rotate90(1) # rotate 90deg cw
		# move back a little before alignment in next step
		
		# STEP 5: Move back a bit to allow alignment in the next step
		backwardShort()
		
		# Next mode is to disninfect the table
		robotMode = 3
		
		
		
	elif robotMode == 3:
		while (robotMode == 3):
			# STEP 1: alignment to table first before continue
			
			# STEP 2: Move forward until leaving table, table not detected by front ToF sensor
			forwardUntilSensor() # Should be moving forward until leaving table, but ToF not available yet
			if stopDisinfection == True:
				robotMode = 1
				continue
			
			# STEP 3: Rotate to the next lane clockwise direction
			rotate90(1)
			forwardShort()
			if stopDisinfection == True:
				robotMode = 1
				break
			rotate90(1)
			
			# STEP 4: alignment to table first before continue (repeat of step 1)
			
			# Step 5: Move forward until leaving table, table not detected by front ToF sensor (repeat of step 2)
			forwardUntilSensor()
			if stopDisinfection == True:
				robotMode = 1
				break
			
			# Step 6: Rotate to the next lane counter-clockwise direction (reverse direction of step 3)
			rotate90(0)
			forwardShort()
			if stopDisinfection == True:
				robotMode = 1
				break
			rotate90(0)
			
			# Loop around to align to table again and continue to disinfect table.
			# Disinfection will stop if entire table has been disinfected, verified by bump and prox sensors
		robotMode = 1 # Next state will be robotMode 1 to look for another table
		stopDisinfection = False # Reset stopDisinfection value
		time.sleep(3)
	    
	    
        # # Reset measurements
	# ser.write('\x8E\x14')
	# time.sleep(.2)
	# while ser.inWaiting() != 0:
		# ser.read()
	# forward()
	# time.sleep(0.1)
        # ser.write('\x8E\x30') #45 = 2D all, 48 = \x30 center left, 49 = \x31 center right
        # time.sleep(.2)
        # while ser.inWaiting() != 0: # receive message from iRobot, instead of polling maybe use interrupts/callbacks
            # response = []
            # for i in range(2):
                # response.append(hex(ord(ser.read())))
            # #print(response)
            # if (int(response[0], 16) > 0 or int(response[1], 16) > 20): # If close, accurate distance not defined
                # rotate90()
                # time.sleep(.2)
            # forward()
         

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
