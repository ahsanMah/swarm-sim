"""my_supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Supervisor, Node, Display

# create the Robot instance.
super = Supervisor()

root = super.getRoot()
emitter = super.getEmitter('emitter')

# emitter.setChannel(emitter.CHANNEL_BROADCAST)

print(super.getFromDef("e-puck"))

names = ["robo_1", "robo_2"]
robots = [super.getFromDef(name) for name in names]

# get the time step of the current world.
timestep = int(super.getBasicTimeStep())
print("Timestep:",timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while super.step(timestep) != -1:
        
        for _id,robot in enumerate(robots):
                pos = robot.getField("translation")
                orientation = robot.getField("rotation")
                
                x,z,y = pos.getSFVec3f()
                current_orientation = orientation.getSFRotation()
                
                data = "SUPERVISOR\t{}\t{}\t{}".format(x,y,current_orientation[3])
                

                # Send only to relevant robot
                emitter.setChannel(_id+1)
                emitter.send(data.encode('utf-8'))

# Multiple Receiver and Emitters with specifoc channels to communicate across 
# different robots
