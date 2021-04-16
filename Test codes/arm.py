import serial
import time 

ser=serial.Serial(port='/dev/ttyS0',
                  baudrate=9600,
                  parity=serial.PARITY_NONE,
                  stopbits=serial.STOPBITS_ONE,
                  bytesize=serial.EIGHTBITS,
                  timeout=1)


ser.write('\x55\x55\x05\x06\x00\x01\x00') #Position ID 0
time.sleep(2)
ser.write('\x55\x55\x05\x06\x02\x01\x00')#Position ID 2
time.sleep(5)
ser.write('\x55\x55\x05\x06\x00\x01\x00') #Position ID 0
time.sleep(2)

ser.write('\x55\x55\x02\x07') #to stop the running action 
ser.close()
