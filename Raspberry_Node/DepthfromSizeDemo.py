from __future__ import print_function

# Import libraries to use
from math import *					# Standard math library
from libpixyusb_swig.pixy import *	# Python wrapper for C++ Pixy CMUcam5 library
from ctypes import *
#import serial						# Library for communicating over serial
#import numpy as np					# Matrix math library
from time import clock, sleep		# Some standard library
import sys


# Pixy Python SWIG get blocks example #

print ("Depth from Size demo-- built on Get Blocks demo")

# Initialize Pixy Interpreter thread #
pixy_init()

class Blocks (Structure):
  _fields_ = [ ("type", c_uint),
               ("signature", c_uint),
               ("x", c_uint),
               ("y", c_uint),
               ("width", c_uint),
               ("height", c_uint),
               ("angle", c_uint) ]

blocks = BlockArray(100)

frame = 0

previousClock = 0

# Wait for blocks #

print("Distance")

while 1:
	
	## Use to store the index of certain colored objects from blocks
	red = []
	yellow = []
	green = []
	purple = []

	count = pixy_get_blocks(100, blocks)
	
	#count = 0
	if count > 0:
		# Blocks found #
		#print 'frame %3d:' % (frame)
		frame = frame + 1
		for index in range (0, count):
			#print '[BLOCK_TYPE=%d SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].type, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height)
			
			## Even though the LED color does not correspond to specific
			## signature colors I have chosen to do just that.
			if blocks[index].signature == 1:		## Red Ball
				red.append(index)
			elif blocks[index].signature == 3:	## Yellow Ball
		#		print '[BLOCK_TYPE=%d SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].type, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height)
				yellow.append(index)
			elif blocks[index].signature == 4:	## Green Ball
				green.append(index)
			elif blocks[index].signature == 7:	## Purple Ball
				purple.append(index)
	
	
	## Removes objects that arent not square to a certain amount
	if red != []:
		temp = list(red)
		for index in range (0,len(red)-1):
			if abs(blocks[red[index]].width - blocks[red[index]].height) > 15:
				temp.remove(red[index])		
		red = list(temp)
		
	if yellow != []:
		temp = list(yellow)
		for index in range (0,len(yellow)-1):
			if abs(blocks[yellow[index]].width - blocks[yellow[index]].height) > 15:
				temp.remove(yellow[index])
		yellow = list(temp)
	
	if green != []:	
		temp = list(green)
		for index in range (0,len(green)-1):
			if abs(blocks[green[index]].width - blocks[green[index]].height) > 15:
				temp.remove(green[index])
		green = list(temp)
		
	if purple != []:
		temp = list(purple)
		for index in range (0,len(purple)-1):
			if abs(blocks[purple[index]].width - blocks[purple[index]].height) > 15:
				temp.remove(purple[index])
		purple = list(temp)
	
	## Sort by Area?
	if red != []:
		temp = list(red)
		for index in range (0,len(red)-1):
			if blocks[red[index]].width*blocks[red[index]].height < 150:
				temp.remove(red[index])
		red = list(temp)
	
	if yellow != []:
		temp = list(yellow)
		for index in range (0,len(yellow)-1):
			if blocks[yellow[index]].width*blocks[yellow[index]].height < 150:
				temp.remove(yellow[index])
		yellow = list(temp)
	
	if green != []:
		temp = list(green)
		for index in range (0,len(green)-1):
			if blocks[green[index]].width*blocks[green[index]].height < 150:
				temp.remove(green[index])
		green = list(temp)
	
	if purple != []:
		temp = list(purple)
		for index in range (0,len(purple)-1):
			if blocks[purple[index]].width*blocks[purple[index]].height < 150:
				temp.remove(purple[index])
		purple = list(temp)
	
	
	redDistance = 0	
	yellowDistance = 0
	greenDistance = 0
	purpleDistance = 0
	
	#for index in yellow:
	#	print '[BLOCK_TYPE=%d SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].type, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height)
	
	## Calculate distance of most significant object 
	if red != []:
		redDistance = 9*104/(blocks[red[0]].width)
	
	if yellow != []:	
		yellowDistance = 9*104/(blocks[yellow[0]].width)
 
	if green != []:	
		greenDistance = 9*104/(blocks[green[0]].width)
		
	if purple != []:	
		purpleDistance = 9*104/(blocks[purple[0]].width)
	
	currentClock = clock()
	if abs(currentClock - previousClock) > .1:	
		previousClock = currentClock
		
		print(purpleDistance)
	
		
"""	
	currentClock = clock()
	if abs(currentClock - previousClock) > .1:	
		previousClock = currentClock
		sys.stdout.write("Red:")
		sys.stdout.flush()
		sys.stdout.write(redDistance)
		print "Yellow:"
		print yellowDistance
		print "Green:"
		print greenDistance
		print "Purple:"
		print purpleDistance
		sys.stderr.write("\x1b[2J\x1b[H")	
"""		
