import serial
import time
import RPi.GPIO as GPIO
from threading  import Timer
import binascii

#setting up serial connection to Raspberry Pi for the Roomba
ser = serial.Serial(port = '/dev/ttyUSB0',baudrate=115200)
ser.flushOutput()

ser.write('\x80')#start mode
time.sleep(0.2)
ser.write('\x83')#safe mode
time.sleep(0.2)
ser.write('\x92\x00\x00\00\00') #wheel speed of 0
ser.timeout = None
leg_close = False

sensor = '\x8E\x2D'
L = '\x8E\x2E'
FL = '\x8E\x2F'
CL = '\x8E\x30'
CR = '\x8E\x31'
FR = '\x8E\x31'
R = '\x8E\x33'

# GPIO initial setup 
#GPIO.setmode(GPIO.BCM)   #set up GPIO pins

# quit button
#GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def check_sensor ():
    global leg_close
    print("Check Sensors")
    ser.write(CL)
    time.sleep(0.01)
    CL_high = ord(ser.read(size=1))
    CL_low = ord(ser.read(size=1))
    print("Center Left: "+str(CL_high)+", "+str(CL_low))
    ser.flushOutput()
    ser.write(CR)
    time.sleep(0.01)
    CR_high = ord(ser.read(size=1))
    CR_low = ord(ser.read(size=1))
    print("Center Right: "+str(CR_high)+", "+str(CR_low))
    ser.flushOutput()
    ser.write(FL)
    time.sleep(0.01)
    FL_high = ord(ser.read(size=1))
    FL_low = ord(ser.read(size=1))
    print("Front Left: "+str(FL_high)+", "+str(FL_low))
    ser.flushOutput()
    ser.write(FR)
    time.sleep(0.01)
    FR_high = ord(ser.read(size=1))
    FR_low = ord(ser.read(size=1))
    print("Front Right: "+str(FR_high)+", "+str(FR_low))
    ser.flushOutput()
    ser.write(L)
    time.sleep(0.01)
    L_high = ord(ser.read(size=1))
    L_low = ord(ser.read(size=1))
    print("Left: "+str(L_high)+", "+str(L_low))
    ser.flushOutput()
    ser.write(FR)
    time.sleep(0.01)
    R_high = ord(ser.read(size=1))
    R_low = ord(ser.read(size=1))
    print("Right: "+str(R_high)+", "+str(R_low))
    ser.flushOutput()

try:
    while True:
        if not leg_close:
            ser.write('\x92\x00\x60\x00\x60') #move forward
            check_sensor()
            tic = time.time()
        if (CL_high+CR_high+FL_high+FR_high+L_high+R_high) or CL_low > 80 or CR_low > 80 or FL_low > 80 or FR_low > 80 or L_low > 80 or R_low > 80:
            print("Close")
            leg_close = True
            toc = time.time()
            if (toc - tic) > 0.5:
                ser.write('\x92\xFF\xA1\x00\x5F') #turn clockwise
            else:
                ser.write('\x92\x00\x00\00\00') #stop
        else:
            leg_clsoe = False
                
except KeyboardInterrupt:
    print("Keyboard Interrupt: exiting")
    ser.write('\x83') #safe mode
    time.sleep(0.2)
    ser.write('\x92\x00\x00\00\00') #speed 0
    time.sleep(0.2)
    ser.write('\xAD') #stop
    time.sleep(0.2)
    ser.close()

