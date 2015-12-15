__Author__ = "Patrick Neal"

"""
 Place intro header here
 Name
 
 Purpose
 
 Any inputs or variable descriptions
"""

from Robot.Robot import *
import struct

# Second import should be beacon locations.
Bob = Robot("Bob", np.array([0, 0, 0]))

# A bunch of logical variables
foundBlock = False
pickedupBlock = False
localizeDone = False
droppedBlock = False
blocksDone = False
setupComplete = False
continuousRun = False
taskComplete = False
arduinoMegaReady = False
arduinoUnoReady = False

# Reset Mega
Bob.arduinoMega.setDTR(level=False)
sleep(0.5)
Bob.arduinoMega.flushInput()
Bob.arduinoMega.setDTR()
sleep(0.5)

# Reset Uno
Bob.arduinoUno.setDTR(level=False)
sleep(0.5)
Bob.arduinoUno.flushInput()
Bob.arduinoUno.setDTR()
sleep(0.5)

print "Waiting for Arduino Setup..."

print 'Mega Handshake: ' + str(struct.unpack("B", Bob.arduinoMega.read())[0])
Bob.arduinoMega.flushOutput()
sleep(0.1)
Bob.arduinoMega.write('s')
sleep(0.1)
Bob.arduinoMega.flushInput()

print 'Uno Handshake: ' + str(struct.unpack("B", Bob.arduinoUno.read())[0])
Bob.arduinoUno.flushOutput()
sleep(0.1)
Bob.arduinoUno.write('s')
Bob.arduinoUno.flushInput()

# Bob should localize first time through
# Bob.localize()

# Robot will run in this loop.
Bob.requestUno()

Bob.requestMega()