


from math import *					# Standard math library
##from libpixyusb_swig.pixy import *	# Python wrapper for C++ Pixy CMUcam5 library
#from ctypes import *
#import serial						# Library for communicating over serial
import numpy as np					# Matrix math library

beacons = np.array([[1, 2],[3, 4]])


print beacons[[0, 0]]