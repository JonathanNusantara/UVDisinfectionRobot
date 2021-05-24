#import packages
import serial
import time
import RPi.GPIO as GPIO
from threading  import Timer
import binascii

#setting up serial connection to Raspberry Pi for the Roomba
ser = serial.Serial(port = '/dev/ttyUSB0',baudrate=115200)
ser.flushOutput()

ser.write(b'\x80')#start mode
time.sleep(0.2)
ser.write(b'\x83')#safe mode
time.sleep(0.2)
ser.write(b'\x92\x00\x00\00\00') #wheel speed of 0
# ser.write('\x92\x00\x60\x00\x60')
# ser.write('\x92\x00\x60\x00\x60')
#ser.write('\x8E\x2D') # Get light bumper sensor\
# ser.write('\x8D\x01') # Get Play sound
# response = ser.readline()
#print(response)
#ser.write('\x8E\x0B') # Get light bumper sensor\
#time.sleep(2)
sensor = b'\x8E\x2D'

#response = ser.read()
#print(response)

#stri = sensor.decode('utf-8')
# while True:
	# ser.write('\x8E\x30') #45 = 2D
	# time.sleep(.2)
	# while ser.inWaiting() != 0:
		# # ser.write('\x8E\x2D') # Get light bumper sensor
		# response = hex(ord(ser.read()))
		# #hexs = binascii.hexlify(response) #.decode('utf-8')
		# #print(response)
		# #print(sensor.decode(decoding='UTF-8', errors='ignore'))
		# if int(response, 16) > 100: # '0x11'
			# print("close")
		# time.sleep(.5)

while True:
	ser.write(b'\x8E\x30') #45 = 2D all, 48 = \x30 center left, 49 = \x31 center right
	time.sleep(.2)
	while ser.inWaiting() != 0:
		response = []
		for i in range(2):
			response.append(hex(ord(ser.read())))
		print(response)
		if int(response[0], 16) > 0 or int(response[1], 16) > 100: # If closse, accurate distance not defined
			print("close")
		time.sleep(.5)
