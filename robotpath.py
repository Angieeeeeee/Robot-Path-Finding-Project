from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait

hub = PrimeHub()

# ------------------------------------------------------------------------------
#  Hardware Initialization 
# ------------------------------------------------------------------------------
leftMotor = Motor(Port.F)
rightMotor = Motor(Port.B)

# ------------------------------------------------------------------------------
#  Global Variables 
# ------------------------------------------------------------------------------
rows = 16
cols = 10 

# Create a blank maze filled with zeros (0 = free tile)
maze = [[0 for _ in range(cols)] for _ in range(rows)]

numObstacles = 13
obstacles = []  # You can fill this in later with (x, y) tuples

# Valid start and end points (x, y) within the grid bounds
start = (0, 0)
end = (cols - 1, rows - 1)

# ------------------------------------------------------------------------------
#  Main (simple test to confirm code loads)
# ------------------------------------------------------------------------------

def forward():
    leftMotor.run(-200)
    rightMotor.run(200)
    wait(3150)
    leftMotor.stop()
    rightMotor.stop()
    wait(100)

def turnRight():
    leftMotor.run(-100)
    rightMotor.run(-100)
    wait(1820)
    leftMotor.stop()
    rightMotor.stop()
    wait(100)

def turnLeft():
    leftMotor.run(100)
    rightMotor.run(100)
    wait(1820)
    leftMotor.stop()
    rightMotor.stop()
    wait(100)