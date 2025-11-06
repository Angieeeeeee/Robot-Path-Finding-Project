from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

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
maze = np.zeros((rows, cols))

numObstacles = 13
obstacles = []
start = 
end = 

# ------------------------------------------------------------------------------
#  Path Finding
# ------------------------------------------------------------------------------


# ------------------------------------------------------------------------------
#  Main
# ------------------------------------------------------------------------------

leftMotor.run(-200)
rightMotor.run(200)
