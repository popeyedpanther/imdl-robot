"""
 Place intro header here
 Name
 
 Purpose
 
 Any inputs or variable descriptions
"""


# Import libraries to use
from math import *
from libpixyusb_swig.pixy import *
import serial
import numpy as np
from time import sleep

# Start serial connection with Arduino (These settings will most likely
# need to be changed).
ArdSerial = serial.Serial(
	port = '/dev/ttyACM0',
	baudrate = 9600, 
)

counter = 32

while True:
	counter += 1
	ArdSerial.write(str(chr(counter)))
	print ArdSerial.readline()
	sleep(0.1)
	if counter == 255:
		counter = 32
	

"""
while True:
	
	
	
	
	if setupComplete:
		continousRun = True
		break

while continousRun:
	
	
	
	
	if shutdownRobot:
		continousRun = False
	
print "Does this work"
"""
