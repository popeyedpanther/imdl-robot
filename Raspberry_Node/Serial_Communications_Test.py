
# Import libraries to use
from math import *					# Standard math library
# from libpixyusb_swig.pixy import *	# Python wrapper for C++ Pixy CMUcam5 library
# from ctypes import *
import serial						# Library for communicating over serial
import struct
import numpy as np					# Matrix math library
from time import clock, sleep		# Some standard library

# Start serial connection with Arduino (These settings will most likely
# need to be changed).

def convertStr(s):
    """Convert string to either int or float."""
    try:
        ret = int(s)
    except ValueError:
        # Try float.
        ret = float(s)
    return ret


# Serial communications for the Arduino Mega
arduinoMega = serial.Serial('/dev/ttyACM1', 9600, timeout = 3, writeTimeout = 3)

# Serial communications for the Arduino Uno
arduinoUno = serial.Serial('/dev/ttyACM0', 9600, timeout = 3, writeTimeout = 3)

# Reset Mega
arduinoMega.setDTR(level=False)
sleep(0.5)
arduinoMega.flushInput()
arduinoMega.setDTR()
sleep(0.5)

# Reset Uno
arduinoUno.setDTR(level=False)
sleep(0.5)
arduinoUno.flushInput()
arduinoUno.setDTR()
sleep(0.5)

# Serial ready variables
arduinoMegaReady = False
arduinoUnoReady = False

print "Waiting for Arduino Setup..."

print 'Mega Handshake: ' + str(struct.unpack("B", arduinoMega.read())[0])
arduinoMega.flushInput()
arduinoMega.write('s')

print 'Uno Handshake: ' + str(struct.unpack("B", arduinoUno.read())[0])
arduinoUno.flushInput()
arduinoUno.write('s')

b = 10.375

Loop = True

while Loop:
    print "This is the Main Menu for the Serial Communications Test"
    print "1. Move a certain amount"
    print "2. Change gripper wrist/claw position"
    print "3. Change pan/tilt angles"
    print "4. Change robot behavior"
    print "5. Request robot state"
    print "Q. Quit (Stops all motion)"
    choice = raw_input()

    if choice == "1":
        while True:
            print "This is the menu for robot movement"
            print "1. Move Forwards"
            print "2. Move Backwards"
            print "3. Turn Left"
            print "4. Turn Right"
            print "5. Stop"
            print "Q. GO back"
            choice = raw_input()

            if choice == '1':
                print "Input desired distance"
                distance = float(raw_input())
                arduinoMega.flushInput()
                arduinoMega.write('9:' + "{:.2f}".format(-distance) + ':' + "{:.2f}".format(distance) + ':' +
                                  '999:999:99:9:9:\r')
                sleep(0.01)
                print arduinoMega.readline()
            elif choice == '2':
                print "Input desired distance"
                distance = float(raw_input())
                arduinoMega.write('9:' + "{:.2f}".format(distance) + ':' + "{:.2f}".format(-distance) + ':' +
                                  '999:999:99:9:9:\r')
                sleep(0.01)
                print arduinoMega.readline()
            elif choice == '3':
                print "Input desired angle"
                amount = convertStr(raw_input())
                distance = b*radians(float(amount))/2
                arduinoMega.write('9:' + "{:.2f}".format(distance) + ':' + "{:.2f}".format(distance) + ':' +
                                  '999:999:99:9:9:\r')
                sleep(0.01)
                print arduinoMega.readline()
            elif choice == '4':
                print "Input desired angle"
                amount = convertStr(raw_input())
                distance = b*radians(float(amount))/2
                arduinoMega.write('9:' + "{:.2f}".format(-distance) + ':' + "{:.2f}".format(-distance) + ':' +
                                  '999:999:99:9:9:\r')
                sleep(0.01)
                print arduinoMega.readline()
            elif choice == '5':
                print "Stopped"
                distance = 0
                arduinoMega.write('9:' + "{:.2f}".format(distance) + ':' + "{:.2f}".format(distance) + ':' +
                                  '999:999:99:9:9:\r')
                sleep(0.01)
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
                if 30 < wrist < 175:
                    arduinoMega.flushInput()
                    arduinoMega.write('9:0:0:' + str(wrist) + ':999:99:9:9:\r')
                    sleep(0.01)
                    print arduinoMega.readline()
                else:
                    print "Invalid Input"
            elif choice == '2':
                print "Input grasp angle in degrees (some limits in place"
                grasp = convertStr(raw_input())
                if 45 <= grasp < 135:
                    arduinoMega.flushInput()
                    arduinoMega.write('9:0:0:999:' + str(grasp) + ':99:9:9:\r')
                    sleep(0.01)
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
                if 0 <=  pan <= 180:
                    arduinoUno.write('9:'+ str(pan) + ':999:\r')
                    sleep(0.1)
                    print arduinoUno.readline()
                else:
                    print "Invalid Input"
            elif choice == '2':
                print "Input tilt angle in degrees (some limits in place)"
                tilt = convertStr(raw_input())
                if 80 < tilt < 130:
                    arduinoUno.write('9:999:' + str(tilt) + ':\r')
                    sleep(0.1)
                    print arduinoUno.readline()
                else:
                    print "Invalid Input"
            elif choice == 'q' or choice == 'Q':
                break

    elif choice == '4':
        while True:
            print "This is the menu for the pan/tilt"
            print "1. Enter Behavior 1-4"
            print "Q. GO back"
            choice = raw_input()

            if choice == '1':
                print "1 = Search,  2 = Pickup,  3 = Drop off,  4 = Localize)"
                behavior = convertStr(raw_input())
                if 1 <= behavior <= 4:
                    arduinoMega.write(str(behavior) + ':99:99:999:999:99:9:9:\r')
                    sleep(0.1)
                    arduinoUno.write(str(behavior) + ':999:999:\r')
                else:
                    print "Invalid Input"
            elif choice == 'q' or choice == 'Q':
                break
                
    elif choice == '5':
        while True:
            print "Requesting Robot State"
            arduinoMega.flushInput()
            arduinoMega.write('9:0:0:999:999:99:1:9:\r')    
            sleep(0.1)
            message = arduinoMega.readline().split(':')
            sleep(0.1)
            arduinoMega.write('9:0:0:999:999:99:9:9:\r')               
            print message

            #print message[0] + ' ' + message[1] + ' ' + message[2]
            
            print "Q. GO back"
            choice = raw_input()

            if choice == 'q' or choice == 'Q':
                break
            



    elif choice == 'q' or choice == 'Q':
        # Send messages to turn off actuators
        arduinoMega.write('9:0:0:180:125:99:9:9:\r')
        arduinoMega.close()
        arduinoUno.write('9:90:90:\r')
        arduinoUno.close()
        Loop = False
