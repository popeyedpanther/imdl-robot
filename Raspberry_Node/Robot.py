__author__ = 'Patrick'

from math import *					# Standard math library
#from libpixyusb_swig.pixy import *	# Python wrapper for C++ Pixy CMUcam5 library
#from ctypes import *
import serial						# Library for communicating over serial
import numpy as np					# Matrix math library
from time import clock, sleep				# Some standard library


class Robot:

    def __init__ (self, robotname, beacons):
        self.name = robotname
        self.behavior = 0
        self.state = np.array([0, 0, 0])
        # Serial communications for the Arduino Mega
        self.arduinoMega = serial.Serial(
	        port = '/dev/ttyACM0',
	        baudrate = 9600)

        # Serial communications for the Arduino Uno
        self.arduinoUno = serial.Serial(
	        port = '/dev/ttyACM1',
	        baudrate = 9600)

        self.beaconPos = beacons



    def stateUpdate(self,statechange):
        self.state += statechange


    def move(self):


    def collectData(self):
        # This needs to collect data from three out of four beacons.
        # These beacons should have a identifier so their specific absolute position is know.
        # Needs to be able to communicate with the Arduino Uno.(Global serial variables?)
        # Also needs to communicate with a Pixy camera
        # Pick the first one that is in view.(Object should meet specific geometry conditions)

        Statetriang = localize(r1,r2,r3,lambda1,lambda2,lambda3)




    def localize(r1,r2,r3,lambda1,lambda2,lambda3):
        # Perform the calculations
        # Convert degrees into radians
        lambda1 = radians(lambda1)
        lambda2 = radians(lambda2)
        lambda3 = radians(lambda3)

        # If less than three beacons return error code
        lambda12 = lambda2 - lambda1
        if lambda1>lambda2: lambda12 += 2*pi

        lambda31 = lambda1 -lambda3
        if lambda3>lambda1: lambda31 += 2*pi

        # Apply check for singularity here
        # if lambda12= 0 or pi or lambda31 = 0 or pi


        # Compute L12 from r1, r2
        L12 = sqrt(()^2 + ()^2)

        # Compute L31 from r1, r3
        L31 = sqrt(()^2 + ()^2)

        # Need to calculate phi and sgima

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