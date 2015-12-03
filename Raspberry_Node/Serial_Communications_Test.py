
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
	
l = 10.375

Loop = True

while Loop:
	print "This is the Main Menu for the Serial Communications Test"
	print "1. Move a certain amount"
	print "2. Change gripper wrist/claw position"
	print "3. Change pan/tilt angles"
	print "Q. Quit (Stops all motion)"
	choice = raw_input()

	if choice == "1":
		while True:
			print "This is the menu for robot movement"
			print "1. Move Forwards"
			print "2. Move Backwards"
			print "3. Turn Left"
			print "4. Turn Right"
			print "Q. GO back"
			choice = raw_input()

			if choice == '1':
				print "Input desired distance"
				amount = convertStr(raw_input())
				arduinoMega.write('9:'+ str(amount)+':'+str(amount)+':'+'999:999:'+'\r')
				sleep(0.1)
				print arduinoMega.readline
			elif choice == '2':
				print "Input desired distance"
				amount = convertStr(raw_input())
				arduinoMega.write('9:'+ str(amount)+':'+str(amount)+':'+'999:999:'+'\r')
				sleep(0.1)
				print arduinoMega.readline()
			elif choice == '3':
				print "Input desired angle"
				amount = convertStr(raw_input())
				distance = l*float(amount)
				arduinoMega.write('9:'+ str(distance)+':'+str(distance)+':'+'999:999:'+'\r')
				sleep(0.1)
				print arduinoMega.readline()
			elif choice == '4':
				print "Input desired angle"
				amount = convertStr(raw_input())
				distance = l*float(amount)
				arduinoMega.write('9:'+ str(-distance)+':'+str(-distance)+':'+'999:999:'+'\r')
				sleep(0.1)
				print arduinoMega.readline()
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
				if wrist < 181 and wrist > 30:
					arduinoMega.write('9:99:99:'+ str(wrist)+':'+'999:'+'\r')
					sleep(0.1)
					print arduinoMega.readline()
				else:
					print "Invalid Input"
			elif choice == '2':
				print "Input grasp angle in degrees (some limits in place"
				grasp = convertStr(raw_input())
				if grasp < 135 and grasp >= 45:
					arduinoMega.write('9:99:99:999:'+ str(grasp)+':'+'\r')
					sleep(0.1)
					print arduinoMega.readline()
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
				if pan <= 180 and pan >=0:
					arduinoUno.write('9:'+ str(pan)+':'+'999:'+'\r')
					sleep(0.1)
					print arduinoUno.readline()
				else:
					print "Invalid Input"
			elif choice == '2':
				print "Input tilt angle in degrees (some limits in place)"
				tilt = convertStr(raw_input())
				if tilt < 130 and tilt > 80:
					arduinoUno.write('9:'+ '999:' +str(tilt) +':' +'\r')
					sleep(0.1)
					print arduinoUno.readline()
				else:
					print "Invalid Input"
			elif choice == 'q' or choice == 'Q':
				break

	elif choice == 'q' or choice == 'Q':
		# Send messages to turn off actuators
		arduinoMega.write('9:0:0:180:50:'+'\r')
		arduinoMega.close()
		arduinoUno.close()
		Loop = False
