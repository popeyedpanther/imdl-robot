__Author__ = "Patrick Neal"

"""
 Place intro header here
 Name
 
 Purpose
 
 Any inputs or variable descriptions
"""

from Robot.Robot import *

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

print 'Mega Handshake: ' + str(struct.unpack("B", arduinoMega.read())[0])
Bob.arduinoMega.flushOutput()
sleep(0.1)
Bob.arduinoMega.write('s')
sleep(0.1)
Bob.arduinoMega.flushInput()

print 'Uno Handshake: ' + str(struct.unpack("B", arduinoUno.read())[0])
Bob.arduinoUno.flushOutput()
sleep(0.1)
Bob.arduinoUno.write('s')
Bob.arduinoUno.flushInput()

# Bob should localize first time through
# Bob.localize()

# Robot will run in this loop.
while continuousRun:

    if not foundBlock:      # Searches for the block
        if Bob.behavior != 1:
            Bob.updateBehavior(1)

    elif foundBlock:        # Picks up the block
        if Bob.behavior != 2:
            Bob.updateBehavior(2)

    elif pickedupBlock:     # Should localize and drop the block
        ## Should cross of that block from search list
        # Bob.localize()

        if Bob.behavior != 3:
            Bob.updateBehavior(3)

    elif droppedBlock:
        # if all blocks are found then task is complete else return to

        if blocksDone:
            taskComplete = True
        else:
            foundBlock = False
            pickedupBock = False
            droppedBlock = False

    if Bob.behavior == 1:
        # Do what is necessary to find a block
        # Issue movement commands to the Arduino Mega
        # Request updates from the Arduino Uno
        Bob.arduinoUno.flushInput()
        Bob.arduinoUno.flushOutput()
        Bob.arduinoUno.write(
    elif Bob.behavior == 2:
        # Do what is necessary to pick up the block
    elif Bob.behavior == 3:

    elif Bob.behavior == 4:


    # Just start moving forward (maybe move randomly to increase chance of seeing block
    Bob.move("F", 25)



    # Poll the Uno and Mega for updates
    Bob.stateUpdate()

    # Update the state of Bob maybe after some specified time.

    # Behavior changes should go here



    if taskComplete:
        # Stop robot functions and then break loop
        break


