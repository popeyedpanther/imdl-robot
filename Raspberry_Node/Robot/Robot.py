__author__ = 'Patrick'

from math import *					# Standard math library
import serial						# Library for communicating over serial
import numpy as np					# Matrix math library
from time import clock, sleep
#from libpixyusb_swig.pixy import *	# Python wrapper for C++ Pixy CMUcam5 library
#from ctypes import *
import random


class Robot:

    def __init__(self, robotName, beacons):
        self.name = robotName               # Name of your robot
        self.behavior = 0                   # Current behavior the robot is in
        self.state = np.array([float(0), float(0), float(0)])
        self.OAState = 0
        self.Pan = 0
        self.Tilt = 0
        self.foundObject = 0
        self.objectX = 0
        self.objectY = 0
        self.motionComplete = 0
        self.OAOverride = 0
        self.blocksDone = 0
        # Serial communications for the Arduino Mega
        self.arduinoMega = serial.Serial(port = '/dev/ttyACM1', baudrate = 9600, timeout = 3, writeTimeout = 3)

        # Serial communications for the Arduino Uno
        self.arduinoUno = serial.Serial(port = '/dev/ttyACM0', baudrate = 9600, timeout = 3, writeTimeout = 3)

        self.arduinoMega.flushInput()
        self.arduinoMega.flushOutput()

        self.arduinoUno.flushInput()
        self.arduinoUno.flushOutput()

        self.beaconPos = beacons

    def stateUpdate(self, message):

        statechange = np.array([float(message[0]), float(message[1]), float(message[2])])

        self.state += statechange

    def updateBehavior(self, behavior):
        self.behavior = behavior
        self.arduinoMega.write(str(self.behavior) + ':' + '0:0:999:999:99:9:9:\r')
        sleep(0.1)
        self.arduinoUno.write(str(self.behavior) + ':' + '9:9:999:999;\r')  # Update with correct Uno message
        sleep(0.1)

    def move(self, Dir, amount):
        # This method includes moving the robot and moving the gripper(wrist and claw) and pan
        b = 10.375  # inches, distance between wheels
        if Dir == 'L' or Dir == 'l':
            distance = b*radians(float(amount))/2
            self.writeMega('9:' + "{:.2f}".format(-distance) + ':' + "{:.2f}".format(distance) + ':' +
                                   '999:999:99:9:9:\r')
        elif Dir == 'R' or Dir == 'r':
            distance = b*radians(float(amount))/2
            self.writeMega('9:' + "{:.2f}".format(distance) + ':' + "{:.2f}".format(-distance) + ':' +
                                   '999:999:99:9:9:\r')
        elif Dir == 'F' or Dir == 'f':
            distance = float(amount)
            self.writeMega('9:' + "{:.2f}".format(distance) + ':' + "{:.2f}".format(distance) + ':' +
                                   '999:999:99:9:9:\r')
        elif Dir == 'B' or Dir == 'b':
            distance = float(raw_input())
            self.writeMega('9:' + "{:.2f}".format(-distance) + ':' + "{:.2f}".format(-distance) + ':' +
                                   '999:999:99:9:9:\r')
        elif Dir == 'W' or Dir == 'w':
            self.writeMega('9:0:0:' + str(amount) + ':999:99:9:9:\r')
        elif Dir == 'C' or Dir == 'c':
            self.writeMega('9:0:0:999:' + str(amount) + ':99:9:9:\r')
        elif Dir == 'P' or Dir == 'p':
            self.writeUno('9:9:9:' + str(amount) + ':999;\r')
        elif Dir == 'S' or Dir == 's':
            self.writeMega('9:0:0:999:999:99:9:9:\r')

    def requestMega(self):
        self.arduinoMega.flushInput()
        self.writeMega('9:0:0:999:999:99:1:9:\r')
        sleep(0.075)
        message = self.arduinoMega.readline().split(':')
        # Parse the message here
        self.stateUpdate([message[0], message[1],  message[2]])
        self.motionComplete = int(message[3])
        self.OAOverride = int(message[4])

    def requestUno(self):
        self.arduinoUno.flushInput()
        self.writeUno('9:1:9:999:999:\r')
        sleep(0.075)
        message = self.arduinoUno.readline().split(':')
        # Parse the message here
        self.Pan = int(message[0])
        self.Tilt = int(message[1])
        self.foundObject = int(message[2])
        self.blocksDone = int(message[3])
        self.objectX = int(message[4])
        self.objectY = int(message[5])

    def writeMega(self, message):
        self.arduinoMega.flushOutput()
        sleep(0.075)
        self.arduinoMega.write(message)
        

    def writeUno(self, message):
        self.arduinoUno.flushOutput()
        sleep(0.075)
        self.arduinoUno.write(message)


"""
def readMega(self):
    # This method will read the output of the Arduino Mega and parse the information
    # returns a list of the separate components of the message as strings
    self.arduinoMega.flushInput()
    sleep(0.1)
    megaMessage = self.arduinoMega.readline()
    messageListMega = megaMessage.split(":")
    return messageListMega

def readUno(self):
    # This method will read the output of the Arduino Uno and parse the information
    # returns a list of the separate components of the message as strings
    self.arduinoUno.flushInput()
    sleep(0.1)
    unoMessage = self.arduinoUno.readline()
    messageListUno = unoMessage.split(":")
    return messageListUno
"""


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
