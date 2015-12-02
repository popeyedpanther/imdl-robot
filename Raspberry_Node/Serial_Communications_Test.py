
# Import libraries to use
from math import *					# Standard math library
# from libpixyusb_swig.pixy import *	# Python wrapper for C++ Pixy CMUcam5 library
# from ctypes import *
import serial						# Library for communicating over serial
import numpy as np					# Matrix math library
from time import clock, sleep		# Some standard library
import sys

# Start serial connection with Arduino (These settings will most likely
# need to be changed).

def convertStr(s):
    """Convert string to either int or float."""
    try:
        ret = int(s)
    except ValueError:
        #Try float.
        ret = -1
    return ret


# Serial communcations for the Arduino Mega
arduinoMega = serial.Serial('/dev/ttyACM0', 9600)

# Serial communcations for the Arduino Uno	
arduinoUno = serial.Serial('/dev/ttyACM1', 9600)
	

Loop = True

while Loop:
	print "This is the Main Menu for the Serial Communications Test"
	print "1. Change Robot driving speeds"
	print "2. Change gripper wrist/claw position"
	print "3. Change pan/tilt angles"
	print "Q. Quit (Stops all motion)"
	choice = raw_input()

	if choice == "1":
		while True:
			print "This is the menu for motor speeds"
			print "1. Change motor speed setpoints"
			print "Q. GO back"
			choice = raw_input()

			if choice == '1':
				print "Input desired speed in in/s (0-10 integers only)"
				speed = convertStr(raw_input())
				if speed == -1:
					print "Invalid Input"
				else:
					print speed
			elif choice == 'q' or choice == 'Q':
				break

	elif choice == '2':
		while True:
			print "This is the menu for the gripper"
			print "1. Change wrist angle"
			print "2. Change claw position"
			print "Q. GO back"
			choice = raw_input()

			if choice == '1':
				print "Input wrist angle in degrees (some limits in place"
				wrist = convertStr(raw_input())
				if wrist < 160 and wrist > 50:
					print wrist
				else:
					print "Invalid Input"
			elif choice == '2':
				print "Input claw angle in degrees (some limits in place"
				claw = convertStr(raw_input())
				if claw < 170 and claw > 30:
					print claw
				else:
					print "Invalid Input"
			elif choice == 'q' or choice == 'Q':
				break

	elif choice == '3':
		while True:
			print "This is the menu for the pan/tilt"
			print "1. Change pan angle"
			print "2. Change tilt angle"
			print "Q. GO back"
			choice = raw_input()

			if choice == '1':
				print "Input pan angle in degrees (some limits in place)"
				pan = convertStr(raw_input())
				if pan < 160 and pan > 50:
					arduinoUno.write(str(pan))
					sleep(0.1)
					print arduinoUno.readline()
				else:
					print "Invalid Input"
			elif choice == '2':
				print "Input tilt angle in degrees (some limits in place)"
				tilt = convertStr(raw_input())
				if tilt < 170 and tilt > 30:
					print tilt
				else:
					print "Invalid Input"
			elif choice == 'q' or choice == 'Q':
				break

	elif choice == 'q' or choice == 'Q':
		# Send messages to turn off actuators
		Loop = False


	"""
	ArdSerial.write(str(chr(counter)))
	print ArdSerial.readline()
	sleep(0.1)
	if counter == 255:
		counter = 32
	
	"""

