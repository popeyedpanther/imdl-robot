__Author__ = "Patrick Neal"

"""
 Place intro header here
 Name
 
 Purpose
 
 Any inputs or variable descriptions
"""

from Robot.Robot import *
import struct

# Stuff
previousClock = 0.0

# A bunch of logical variables
pickedupObject = False
localizeDone = False
droppedBlock = False
blocksDone = False
taskComplete = False
robotAligned = False

# Second import should be beacon locations.
Bob = Robot("Bob", np.array([0, 0, 0]))

#-------------------------------------------------------Setup-----------------------------------------------------------
#-----------------------------------------------------------------------------------------------------------------------
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

#----------------------------------------------------Main  Loop---------------------------------------------------------
#-----------------------------------------------------------------------------------------------------------------------

# Bob should localize first time through
# Bob.localize()

continuousRun = True

while continuousRun:

    currentClock = clock()

    # Update information from the Arduinos
    if currentClock - previousClock > 1:
        previousClock = currentClock
        Bob.requestMega()
        Bob.requestUno()
        print Bob.OAOverride

    # Behavior changes
    if Bob.foundObject != 1:      # Searches for the block
        if Bob.behavior != 1:
            Bob.updateBehavior(1)
            robotAligned = False
            print 'Searching'

    elif Bob.foundObject == 1 and not pickedupObject:        # Picks up the block
        if Bob.behavior != 2:
            Bob.updateBehavior(2)
            if not Bob.OAOverride:
                Bob.move('S', 0)
            print "I found an object"

    elif pickedupObject and not droppedBlock:     # Should localize and drop the block
        # Should cross of that block from search list
        # Bob.localize()

        if Bob.behavior != 3:
            Bob.updateBehavior(3)
            print "I Dropped the block"
        
    elif droppedBlock:
        # if all blocks are found then task is complete else return to

        if blocksDone:
            taskComplete = True
        else:
            Bob.foundBlock = 0
            pickedupObject = False
            droppedBlock = False

    # Where the actions for each behavior take place
    if Bob.behavior == 1:
        # Do what is necessary to find a block
        # Issue movement commands to the Arduino Mega
        # Perform  some random movements here/drive around
        if not Bob.OAOverride and Bob.motionComplete == 1:
            randDirection = random.randint(1, 9)
            if 1 <= randDirection <= 3:
                # Go forward some amount
                Bob.move('F', random.randint(10, 20))

            elif 4 <= randDirection <= 5:
                # Reverse some amount
                Bob.move('R', random.randint(10, 20))

            elif 5 <= randDirection <= 7:
                # Turn left some amount
                Bob.move('L', random.randint(20, 45))

            else:
                # Turn right some amount
                Bob.move('R', random.randint(20, 45))

    elif Bob.behavior == 2:
        # Do what is necessary to pick up the block
        # Align with the block in x direction and then the y direction
        xError = (160 - Bob.objectX)

        if(abs(xError)) > 15:
            xError = copysign(1, xError)*15

        if copysign(1, xError) > 0 and Bob.motionComplete == 1:
            Bob.move('R', abs(xError))
        elif copysign(1, xError) < 0 and Bob.motionComplete == 1:
            Bob.move('L', abs(xError))

        if 145 < Bob.objectX < 180:
            # Robot is aligned
            robotAligned = True

        # Then go to pick it up
        if robotAligned:
            Bob.move('W', 60)
            sleep(0.2)
            Bob.move('C', 55)
            sleep(0.2)

            # Drive forward and pick up object
            Bob.move('F', 6)
            sleep(1)
            # Close gripper and lift
            Bob.move('C', 132)
            sleep(0.2)
            Bob.move('W', 160)
            sleep(0.2)
            pickedupObject = True

    elif Bob.behavior == 3:
        droppedBlock = True
    elif Bob.behavior == 4:
        taskComplete = True
    if taskComplete:
        # Stop robot functions and then break loop
        break


