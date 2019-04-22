"""e_puck_avoid_collision controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Robot, DistanceSensor, Motor
import numpy as np

# TIME_STEP = 16
MAX_SPEED = 6.28
MAP_X = 1
MAP_Y = 1
CELL_SIZE = 0.01
RADIUS = 3.6 # cm
MAX_VISION = 7
GRID = None


# Distance Map
DISTANCE = [
          [0, 4095],
          [1, 2211],
          [2, 676],
          [3, 306],
          [4, 164],
          [5, 90],
          [6, 56],
          [7, 34]]

# MAP = {"

def init_grid(X,Y,C):
    map_range = [0,500]
    mid_range = (map_range[1] - map_range[0]) // 2
    
    # Initializing grid to middle of map range
    grid = np.zeros(shape=[int(X/C),int(Y/C)], dtype = int)
    grid += mid_range
    return grid



def convert_to_distance(sensor_val):
    d = 0
    for _map in DISTANCE:
        if sensor_val < _map[1]:
            d = _map[0]
    return d
        

def clip(x, y_axis=False):
    x = max(0,x)
    
    dim_max = MAP_Y/CELL_SIZE if y_axis else MAP_X/CELL_SIZE
    x = min(x, int(dim_max))

    return x

def update_map(x,y, sensor_val, quadrant=0):
    
    if quadrant == 0:
      x_min, x_max = x, clip(x + MAX_VISION)
      y_min, y_max = y, cliP(y + MAX_VISION)
    
    for i in range(x_min, x_max):
        for j in range(y_min,y_max):
            
            # If not under robot AND less than MAX_VISION distance away
            dist_from_robot = (i-x)**2 + (j - y)**2
            
            if dist_from_robot > r and dist_from_robot < d :
                GRID[i,j] += 1



# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
print("Timestep:",timestep)


# Init grid
GRID = init_grid(MAP_X, MAP_Y, CELL_SIZE)

# Have to enable each distance sensor
sensors = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

# Set the rate at whoch the sensor should be probed
for i in range(8):
    sensors.append(robot.getDistanceSensor(psNames[i]))
    sensors[i].enable(timestep)

# Getting motors and devices
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
receiver = robot.getReceiver('receiver')

#Initializing motors
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

receiver.enable(timestep)


print(sensors[0].getMinValue())
print(sensors[0].getMaxValue())
# Main loop:
# feedback loop: step simulation until receiving an exit event
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:

    psValues = []
    for i in range(8):
        psValues.append(sensors[i].getValue())
    
     # Read position sent by supervisor.
    
    print("Queue:",receiver.getQueueLength())
    if receiver.getQueueLength() > 0:
        message = receiver.getData().decode('utf-8')
        # dataList=struct.unpack("chd",message)
    
        print("Current Position: " + message)
        receiver.nextPacket()   
    
    # Processing sensor data here
    # detect obstacles
    right_obstacle = psValues[0] > 70.0 or psValues[1] > 70.0 or psValues[2] > 70.0
    
    print("Distance: {}".format(convert_to_distance(psValues[0])))
     
    ## Update Grid 
    
    left_obstacle = psValues[5] > 70.0 or psValues[6] > 70.0 or psValues[7] > 70.0
    
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  += 0.5 * MAX_SPEED
        rightSpeed -= 0.5 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  -= 0.5 * MAX_SPEED
        rightSpeed += 0.5 * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

# Enter here exit cleanup code.
del robot
print("Robot object deleted")
