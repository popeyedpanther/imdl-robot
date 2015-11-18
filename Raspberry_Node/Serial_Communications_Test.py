
# Import libraries to use
from math import *					# Standard math library
from libpixyusb_swig.pixy import *	# Python wrapper for C++ Pixy CMUcam5 library
from ctypes import *
import serial						# Library for communicating over serial
import numpy as np					# Matrix math library
from time import clock, sleep				# Some standard library

# Start serial connection with Arduino (These settings will most likely
# need to be changed).

# Serial communcations for the Arduino Mega
bottomSerial = serial.Serial(
	port = '/dev/ttyACM0',
	baudrate = 9600, 

# Serial communcations for the Arduino Uno	
topSerial = serial.Serial(
	port = '/dev/ttyACM1',
	baudrate = 9600, 
	

counter = 32

while True:
	counter += 1
	ArdSerial.write(str(chr(counter)))
	print ArdSerial.readline()
	sleep(0.1)
	if counter == 255:
		counter = 32
	

