"""my_supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Supervisor, Node

# create the Robot instance.
super = Supervisor()

root = super.getRoot()
emitter = super.getEmitter('emitter')

emitter.setChannel(emitter.CHANNEL_BROADCAST)

print(super.getFromDef("e-puck"))

robot = super.getFromDef("Robo1")

# get the time step of the current world.
timestep = int(super.getBasicTimeStep())
print("Timestep:",timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  led = robot.getLED('ledname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while super.step(timestep) != -1:
        
        pos = robot.getField("translation")
        x,z,y = pos.getSFVec3f()
        # print(x*100,round(y*100))
        data = "{},{}".format(x,y)
        print(data)
        emitter.send(data.encode('utf-8'))
# Read the sensors:
# Enter here functions to read sensosr data, like:
#  val = ds.getValue()

# Process sensor data here.

# Enter here functions to send actuator commands, like:
#  led.set(1)
# pass

# Multiple Receiver and Emitters with specifoc channels to communicate across 
# different robots
