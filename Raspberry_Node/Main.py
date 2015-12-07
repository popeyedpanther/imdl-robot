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
				if arduinoUno.readline() == 'g\r\n':
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
Bob.arduinoUno.flushOutpu()

# Robot will run in this loop.
while continuousRun:
    # Bob should localize first time through

    # Just start moving forward (maybe move randomly to increase chance of seeing block
    Bob.move("F", 25)

    # Poll the Uno and Mega for updates
    Bob.stateUpdate()

    # Update the state of Bob maybe after some specified time.

    # Behavior changes should go here



    if taskComplete:
        # Stop robot functions and then break loop
        break


