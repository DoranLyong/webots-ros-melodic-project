"""ure_grasper_tutorial controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# 
# (refer to this code): https://github.com/cyberbotics/webots/blob/master/projects/robots/universal_robots/controllers/ure_can_grasper/ure_can_grasper.c


from controller import Robot
from controller import DistanceSensor #(ref) https://cyberbotics.com/doc/reference/distancesensor?tab-language=python
from controller import Motor # (ref) https://cyberbotics.com/doc/reference/motor?tab-language=python
from controller import PositionSensor # (ref) https://cyberbotics.com/doc/reference/positionsensor?tab-language=python 

counter = 0 


target_positions = [-1.88, -2.14, -2.38, -1.51]
speed = 0.5 


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())



# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
hand_motor_joints = ["finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"]
hand_motors = [] 
for hand_motor in hand_motor_joints :
    """ (ref) https://cyberbotics.com/doc/guide/gripper-actuators#robotiq-3f-gripper
    """
    hand_motors.append(robot.getDevice(hand_motor))  

    

ur_motor_joints = ["shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint"]
ur_motors = [] 
for ur_motor in ur_motor_joints:
    """(ref) https://cyberbotics.com/doc/guide/ure?tab-language=python
    """
    ur_motors.append(robot.getDevice(ur_motor))
    hand_motors[-1].setVelocity(speed) #(ref) https://cyberbotics.com/doc/reference/motor?tab-language=python#motor-functions


distance_sensor = robot.getDevice("distance sensor")
distance_sensor.enable(timestep) # (ref) https://cyberbotics.com/doc/reference/distancesensor?tab-language=c#distancesensor-functions

position_sensor = robot.getDevice("wrist_1_joint_sensor")
position_sensor.enable(timestep) # (ref) https://cyberbotics.com/doc/reference/positionsensor?tab-language=python#positionsensor-functions





# Main loop:
# - perform simulation steps until Webots is stopping the controller

state = 'WAITING' # init state  
print(state)

while robot.step(timestep) != -1:
    if counter <= 0 : 
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        if state == 'WAITING':
            if (distance_sensor.getValue() < 500 ): #(ref) https://cyberbotics.com/doc/reference/distancesensor?tab-language=python#distancesensor-functions
                state = "GRASPING"
                counter = 8 

                print("Grasping can \n")                
                for i in range(len(hand_motor_joints)): 
                    hand_motors[i].setPosition(0.85) # (ref) https://cyberbotics.com/doc/reference/motor?tab-language=python#motor-functions
        
        elif state == 'GRASPING':
            for j in range(len(ur_motor_joints)):
                ur_motors[j].setPosition(target_positions[j])

            print("Rotating arm\n")
            state = "ROTATING"


        elif state == "ROTATING":
            if (position_sensor.getValue() < -2.3): # (ref) https://cyberbotics.com/doc/reference/positionsensor?tab-language=python#positionsensor-functions
                counter = 8
                print("Releasing object\n")
                state = "RELEASING"
                for k in range(len(hand_motor_joints)):
                    hand_motors[k].setPosition(hand_motors[k].getMinPosition()) # (ref) https://cyberbotics.com/doc/reference/motor?tab-language=python#motor-functions


        elif state == "RELEASING":
            for x in range(len(ur_motor_joints)):                      
                ur_motors[x].setPosition(0.0)

            print("Rotating arm back \n") 
            state = "ROTATING_BACK"

        elif "ROTATING_BACK":
            if (position_sensor.getValue() > -0.1):
                state = 'WAITING'
                print("Waiting object \n")


    counter -= 1
                
                            
# Enterhere exit cleanup code.
