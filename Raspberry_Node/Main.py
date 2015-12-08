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

print "Waiting for Arduino Setup"

while True:

    # Wait for the Arduino Mega to respond ready
    if not arduinoMegaReady:
        messageMega = Bob.arduinoMega.readline()
        print messageMega
        if messageMega == 'r\r\n':
            while True:
                Bob.arduinoMega.write('s')
                sleep(0.01)
                if Bob.arduinoMega.readline() == 'g\r\n':
                    break
            arduinoMegaReady = True
            print "Mega Done"

    sleep(0.1)

    # Wait for the arduino Uno to respond ready
    if not arduinoUnoReady:
        messageUno = Bob.arduinoUno.readline()
        print messageUno
        if messageUno == 'r\r\n':
            while True:
                Bob.arduinoUno.write('s')
                sleep(0.01)
                if Bob.arduinoUno.readline() == 'g\r\n':
                    break
            arduinoUnoReady = True
            print "Uno Done"
        sleep(0.1)

    # Leave setup when both arduinos are ready
    if arduinoMegaReady and arduinoUnoReady:
        break

# Clear the serial buffers before main loop
Bob.arduinoMega.flushInput()
Bob.arduinoMega.flushOutput()

Bob.arduinoUno.flushInput()
Bob.arduinoUno.flushOutput()

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


