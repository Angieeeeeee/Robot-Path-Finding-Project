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
rows = 10
cols = 16

# Create a blank maze filled with zeros (0 = free tile)
maze = [[0 for _ in range(cols)] for _ in range(rows)]

numObstacles = 13
obstacles = [(1, 1), (1, 2), (2, 1), (2, 2), (2, 7), (2, 8), (5, 4), 
(5, 5), (5, 6), (8, 2), (8, 8), (9, 2), (10, 2)]

# Valid start and end points (x, y) within the grid bounds
start = (1, 5)
end = (10, 5)

# ------------------------------------------------------------------------------
#  Main (simple test to confirm code loads)
# ------------------------------------------------------------------------------

def forward():
    leftMotor.run(-200)
    rightMotor.run(200)
    wait(3050)
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

