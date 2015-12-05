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
unoReady = False
megaReady = False
taskComplete = False

while True:
    # Use bob methods to communicate with the arduinos
    if not unoReady:
        unoReady = Bob.unoSetup()

    if not megaReady:
        megaReady = Bob.megaSetup()


    if unoReady and megaReady:
        continuousRun = True
        break

while continuousRun:
    # Bob should localize first time through

    Bob.move("F", 25)

    if taskComplete:
        # Stop robot functions and then break loop
        break


