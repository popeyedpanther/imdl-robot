__author__ = 'Patrick'

from math import *					# Standard math library
from libpixyusb_swig.pixy import *	# Python wrapper for C++ Pixy CMUcam5 library
from ctypes import *
import serial						# Library for communicating over serial
import numpy as np					# Matrix math library
from time import clock, sleep				# Some standard library


class Robot():

    def __init__ (self):
        self.state = np.array([0, 0, 0])
        # Setup serial communications?
        bottomSerial = serial.Serial(
	        port = '/dev/ttyACM0',
	        baudrate = 9600,

        # Serial communcations for the Arduino Uno
        topSerial = serial.Serial(
	        port = '/dev/ttyACM1',
	        baudrate = 9600,

    def move(self):

    def sense(self):

    def obstacleAvoidance(self):

