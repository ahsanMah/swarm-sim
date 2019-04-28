"""e_puck_avoid_collision controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Robot, DistanceSensor, Motor, Display
import numpy as np
from matplotlib import pyplot as plt
plt.rcParams['axes.labelsize'] = 16
plt.rcParams['axes.titlesize'] = 18
plt.rcParams['xtick.labelsize'] = 14
plt.rcParams['ytick.labelsize'] = 14

# TIME_STEP = 16
MAX_SPEED = 6.28
MAP_X = 1
MAP_Y = 1
CELL_SIZE = 0.01
RADIUS = 3.6 # cm
MAX_VISION = 7
GRID = None
MAP_RANGE = [0,500]
MAX_CHARGE = 127
CHARGE_INCREMENT = 10

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
    
    mid_range = (MAP_RANGE[1] - MAP_RANGE[0]) // 2
    
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
    
    # List of all cells that were updated by robot
    update_list = []
    
    if quadrant == 0:
      x_min, x_max = x, clip(x + MAX_VISION)
      y_min, y_max = clip(y - MAX_VISION), y
    
    for i in range(x_min, x_max):
        for j in range(y_min,y_max):
            
            # If not under robot AND less than observed sensor distance away
            dist_from_robot = (i-x)**2 + (j - y)**2
            
            under_robot = dist_from_robot < RADIUS
            free_space = dist_from_robot < sensor_val 

            if not under_robot and free_space :
                GRID[i,j] += CHARGE_INCREMENT
                update_list.append("{},{}".format(i,j))
                # print("FREESPACE:{},{}".format(i,j))
    return update_list

def print_map(name="map.png", snapshot=1):
    fig = plt.figure(figsize=(20,20))
    plt.imshow(GRID, cmap="gray", 
               vmin= MAP_RANGE[0], vmax= MAP_RANGE[1])
    plt.colorbar()
    plt.title("Potential Map")
    plt.savefig("figures/{id}/{snapshot}_{name}".format(id = ID, name=name, snapshot=snapshot))
    plt.close()


def calculate_potential(robot_pos):
    F = 0 # Approximating the electrostatic Force
    robot_pos = np.array(robot_pos)
    r_x, r_y = robot_pos

    x_min, x_max = clip(r_x - MAX_VISION), clip(r_x + MAX_VISION)
    y_min, y_max = clip(r_y - MAX_VISION, y_axis=True), clip(r_y + MAX_VISION, y_axis=True)

    # print("xmin,xmax: ", x_min,x_max)
    # print(y_min, y_max)

    for x in range(x_min, x_max):
        for y in range(y_min, y_max):
            q = MAX_CHARGE #V[x,y]
            dist_from_robot = (r_x - x)**2 + (r_y - y)**2
            
            if dist_from_robot > RADIUS:
                r = robot_pos - [x,y]
                mag = np.power(np.linalg.norm(r),2)
                F += (q/mag * r)
    return F
       

def calculate_angle(F,orientation):
    phi = np.arctan(-F[1]/F[0])
    print("PHI:",phi)
    return phi - (orientation + np.pi/2)
    
def calculate_speed(angle):
    
    #Steering right
    rightSpeed = 0.5 * MAX_SPEED
    leftSpeed = MAX_SPEED
    
    #Steer left
    if (angle < -np.pi and angle > -2*np.pi) or (angle > 0 and angle < np.pi):
        rightSpeed = MAX_SPEED
        leftSpeed = 0.5 * MAX_SPEED
        print("Steering Left...")

    return rightSpeed, leftSpeed

def parse_message(message):

    message = message.split("\t")
    print("{id}: {msg}".format(id=ID, msg=message))
    data = []
    supervisor = False

    if message[0] == "SUPERVISOR":
        data = [float(x) for x in message[1:]]
        x_pos,y_pos = [int(x*100) for x in data[:2]]
        x_pos += GRID.shape[0]/2
        y_pos += GRID.shape[1]/2
        orientation = data[-1]
        data = [x_pos,y_pos,orientation]
        supervisor = True
    else:
        # List of cells that were visited by other robot
        data = [(int(cell[0]),int(cell[1])) for cell in message[1:]] 
    
    return data, supervisor

    
def synchronize(update):

    for (i,j) in update:
        GRID[i,j] += CHARGE_INCREMENT


# Create the Robot instance.
robot = Robot()

# Get ID
ID = int(robot.getName().split("_")[1])

#GETS THE DISPLAY WORKS!
# disp = robot.getDisplay("display")
# print("DISP WIDTH")
# print(disp.getWidth())
# disp.setColor(0xFF0FFF)
# disp.drawRectangle()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
print("Timestep:",timestep)


# Init grid
GRID = init_grid(MAP_X, MAP_Y, CELL_SIZE)

# Saving copy of initial grid
GRID_0 = GRID.copy()

V = np.zeros((GRID.shape[0],GRID.shape[1])) + MAX_CHARGE
# R =  

# Have to enable each distance sensor
sensors = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

# Set the rate at which the sensor should be probed
for i in range(8):
    sensors.append(robot.getDistanceSensor(psNames[i]))
    sensors[i].enable(timestep)

# Getting motors and devices
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
receiver = robot.getReceiver('receiver')
emitter = robot.getEmitter("emitter")

#Initializing motors
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
rightSpeed, leftSpeed = MAX_SPEED, MAX_SPEED

receiver.enable(timestep)
receiver.setChannel(ID)
emitter.setChannel(emitter.CHANNEL_BROADCAST)

x_pos, y_pos = [0,0]


# Main loop:
# feedback loop: step simulation until receiving an exit event
# - perform simulation steps until Webots is stopping the controller
snapshot_timer = 0
while robot.step(timestep) != -1:
    # Read the sensors:

    psValues = []
    for i in range(8):
        psValues.append(
            convert_to_distance(
            sensors[i].getValue()
            ))
    
     # Read data sent by other robot entities
    if receiver.getQueueLength() > 0:
        message = receiver.getData().decode('utf-8')
        receiver.nextPacket()   
        
        data, supervisor = parse_message(message)

        # Supervisor only sends position coordinates
        if supervisor:
            x_pos,y_pos,orientation = data
            print("Position:{},{} Orientation: {:.4f}".format(x_pos,y_pos, orientation))
        
            ## Update Grid
            update_list = update_map(x= x_pos,
                                     y= y_pos,
                                     sensor_val=psValues[0])
            
            outbound_pkt = "ID_{}:\t".format(ID)
            outbound_pkt += "\t".join(update_list)
            print(outbound_pkt)

            # Send updates to other robots 
            emitter.send(outbound_pkt.encode('utf-8'))
        else:
            # Synchronize grid wioth received updates
            synchronize(data)
            
        ## Calculate the potential force
        # TODO: If first packet received is not from robot, xpo and ypos are incorrect
        F = calculate_potential(robot_pos=[x_pos,y_pos])
        theta = calculate_angle(F, orientation)
        print("Force: {}, Theta: {:.5f}".format(F,theta))

        # Modify speeds according to potentials
        rightSpeed, leftSpeed = calculate_speed(theta)

        # Every hundred timesteps, print a recording of the map
        if snapshot_timer % (timestep * 10) == 0:
            # snapshot_timer = 0
            print_map(snapshot= int(snapshot_timer/10))



    # Processing sensor data here to detect obstacles
    obstacle = psValues[0] < 2 or psValues[7] < 2
    if obstacle:
        print("OBSTACLE!")
        rightSpeed, leftSpeed = MAX_SPEED, -MAX_SPEED

    print("Distance: {}".format(psValues[0]))
    
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    snapshot_timer += timestep

# Enter here exit cleanup code.
del robot
print("Robot object deleted")
