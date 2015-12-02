"""
 Place intro header here
 Name
 
 Purpose
 
 Any inputs or variable descriptions
"""


# Import libraries to use
from math import *					# Standard math library
# from libpixyusb_swig.pixy import *	# Python wrapper for C++ Pixy CMUcam5 library
# from ctypes import *
# import serial						# Library for communicating over serial
import numpy as np					# Matrix math library
from time import clock, sleep				# Some standard library
import Robot


# Second import should be beacon locations.
RobotObject = Robot("Bob", np.array([0, 0, 0]))

Robot.collectData()



while True:
	counter += 1
	ArdSerial.write(str(chr(counter)))
	print ArdSerial.readline()
	sleep(0.1)
	if counter == 255:
		counter = 32
	


"""
# Test code for reading in Pixy information
print ("Pixy Python SWIG Example -- Get Blocks")

# Initialize Pixy Interpreter thread #
pixy_init() # One camera
#pixy_init() # Another camera?


class Blocks (Structure):
  _fields_ = [ ("type", c_uint),
               ("signature", c_uint),
               ("x", c_uint),
               ("y", c_uint),
               ("width", c_uint),
               ("height", c_uint),
               ("angle", c_uint) ]

blocks = BlockArray(100)
frame  = 0

# Wait for blocks #
while 1:

  count = pixy_get_blocks(100, blocks)

  if count > 0:
    # Blocks found #
    print 'frame %3d:' % (frame)
    frame = frame + 1
    for index in range (0, count):
      print '[BLOCK_TYPE=%d SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].type, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height)
"""


## ---Initiliaze Here---
# Initialize Pixy Interpreter thread #
pixy_init() # One camera

# Class to hold incoming blocks from the Pixy CMUcam5
class Blocks (Structure):
  _fields_ = [ ("type", c_uint),
               ("signature", c_uint),
               ("x", c_uint),
               ("y", c_uint),
               ("width", c_uint),
               ("height", c_uint),
               ("angle", c_uint) ]

blocks = BlockArray(100)
frame  = 0

# Timing Variables
currentTime = 0
previousPixyTime = 0


while True:
	
	
	
	
	if setupComplete:
		continousRun = True
		break

while continousRun:


