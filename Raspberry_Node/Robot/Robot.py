__author__ = 'Patrick'

from math import *					# Standard math library
#from libpixyusb_swig.pixy import *	# Python wrapper for C++ Pixy CMUcam5 library
#from ctypes import *
import serial						# Library for communicating over serial
import numpy as np					# Matrix math library
from time import clock, sleep				# Some standard library
import random


class Robot:

    def __init__(self, robotName, beacons):
        self.name = robotName               # Name of your robot
        self.behavior = 0                   # Current behavior the robot is in
        self.state = np.array([0, 0, 0])
        # Serial communications for the Arduino Mega
        self.arduinoMega = serial.Serial(port = '/dev/ttyACM0', baudrate = 9600, timeout = 1, writeTimeout = 2)

        # Serial communications for the Arduino Uno
        self.arduinoUno = serial.Serial(port = '/dev/ttyACM1', baudrate = 9600, timeout = 1, writeTimeout = 2)

        self.arduinoMega.flushInput()
        self.arduinoMega.flushOutput()

        self.arduinoUno.flushInput()
        self.arduinoUno.flushOutput()

        self.beaconPos = beacons

    def stateUpdate(self):
        self.arduinoMega.write()
        sleep(0.1)
        message = self.arduinoMega.readline()

        statechange = np.array([float(message[0]), float(message[1]), float(message[2])])

        self.state += statechange

    def updateBehavior(self, behavior):
        self.behavior = behavior
        self.arduinoMega.write(str(self.behavior) + ':' + '99:99:999:999:99:9:9:\r')
        sleep(0.1)
        self.arduinoUno.write(str(self.behavior) + ':' + '99:99:999:999:99:9:9:\r')  # Update with correct Uno message
        sleep(0.1)

    def move(self, Dir, amount):
        b = 10.375  # inches, distance between wheels
        if Dir == 'L' or Dir == 'l':
            distance = b*float(amount)/2
            self.arduinoMega.write('9:' + "{:.2f}".format(distance) + ':' + "{:.2f}".format(distance) + ':' +
                                   '999:999:99:9:9:\r')
        elif Dir == 'R' or Dir == 'r':
            distance = b*float(amount)/2
            self.arduinoMega.write('9:' + "{:.2f}".format(-distance) + ':' + "{:.2f}".format(-distance) + ':' +
                                   '999:999:99:9:9:\r')
        elif Dir == 'F' or Dir == 'f':
            distance = float(amount)
            self.arduinoMega.write('9:' + "{:.2f}".format(-distance) + ':' + "{:.2f}".format(distance) + ':' +
                                   '999:999:99:9:9:\r')
        elif Dir == 'B' or Dir == 'b':
            distance = float(raw_input())
            self.arduinoMega.write('9:' + "{:.2f}".format(distance) + ':' + "{:.2f}".format(-distance) + ':' +
                                   '999:999:99:9:9:\r')

    def readMega(self):
        # This method will read the output of the Arduino Mega and parse the information
        # returns a list of the separate components of the message as strings
        megaMessage = self.arduinoMega.readline()
        messageListMega = megaMessage.split(":")
        return messageListMega

    def readUno(self):
        # This method will read the output of the Arduino Uno and parse the information
        # returns a list of the separate components of the message as strings
        unoMessage = self.arduinoUno.readline()
        messageListUno = unoMessage.split(":")
        return messageListUno
    """
    def collectdata(self):
        # This needs to collect data from three out of four beacons.
        # These beacons should have a identifier so their specific absolute position is know.
        # Needs to be able to communicate with the Arduino Uno.(Global serial variables?)
        # Also needs to communicate with a Pixy camera
        # Pick the first one that is in view.(Object should meet specific geometry conditions)

        # Tell both the Mega and Uno what behavior is now acting (Localize)

        # Arduino Mega should stop movements and passive while waiting for more commands.
        self.arduinoMega.write()
        self.arduinoUno.write()





        foundBeacons = False

        # Decides where to start looking left or right.
        if random.randint(0, 9)/5 == 1:
            # Start turning left
            direction = 'L'
        else:
            direction = 'R'

        lambdaArray = np.array([0, 0, 0])

        # Read the current state of the
        self.arduinoUno.read()

        while not foundBeacons:
            # Find first beacon ( start randomly looking left or right first find the closest.)
            # Add the beacon ID to a list along with the angle *maybe numpy array)
            # continue in the same direction to find the next

            # If at boundary then command the robot to rotate some amount( with move command?)
            # Also rotate the camera back the same amount.

            # Continue measuring angle between next beacon and place in the list


            if beaconCount == 3:
                foundBeacons = True

            if  and direction == 'L' and not foundBeacons:




         = self.localize(r1, r2, r3, lambdaArray[0], lambdaArray[1], lambdaArray[2])

        # Some error checking maybe

        # Should return Data which stores the measured angles

        return Data




    def localize(self):

        # Update robot current behavior
        self.updateBehavior(4)

        # Collect data for localizng
        Angles = self.collectData()

        # Perform the calculations


        # Convert degrees into radians
        lambda1 = radians(lambda1)
        lambda2 = radians(lambda2)
        lambda3 = radians(lambda3)

        # If less than three beacons return error code
        lambda12 = lambda2 - lambda1
        if lambda1 > lambda2: lambda12 += 2*pi

        lambda31 = lambda1 - lambda3
        if lambda3 > lambda1: lambda31 += 2*pi

        # Apply check for singularity here
        # if lambda12= 0 or pi or lambda31 = 0 or pi


        # Compute L12 from r1, r2
        L12 = sqrt(()^2 + ()^2)

        # Compute L31 from r1, r3
        L31 = sqrt(()^2 + ()^2)

        # Need to calculate phi and sigma

        # Calculate gamma angle
        gamma = sigma - lambda31

        tau = atan((sin(lambda12)*(L12*sin(lambda31)-L31*sin(gamma)))/(L31*sin(lambda12)*cos(gamma)-L12*cos(lambda12)*sin(lambda31)))

        if lambda12 < pi and tau < 0: tau += pi
        if lambda12 > pi and tau > 0: tau -= pi

        if abs(sin(lambda12))>abs(sin(lambda31)):
            L1 = (L12*sin(tau+lambda12))/sin(lambda12)
        else
            L1 = (L31*sin(tau+sigma-lambda31))/sin(lambda31)

        xR = r1.x - L1*cos(phi+tau)
        yR = r1.y - L1*sin(phi+tau)

        thetaR = phi +tau -lambda1
        if thetaR <= -pi:
            thetaR += 2*pi
        elif thetaR>pi:
            thetaR -= 2*pi

        stateTriang =  np.array([xR, yR, thetaR])
        self.state = stateTriang
"""
"""
    def unoSetup(self):
        # Uno should not return ready unless button is pressed.
        message = self.arduinoUno.readline()

        if message == "r":
            ready = True
            self.arduinoUno.write("s" + "\r")
        else:
            ready = False

        return ready

    def megaSetup(self):

        message = self.arduinoMega.readline()

        if message == "r":
            Ready = True
            self.arduinoMega.write("s" + "\r")
        else:
            Ready = False

        return Ready
"""
