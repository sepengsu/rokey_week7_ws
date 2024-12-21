import numpy as np
from visual_kinematics.RobotSerial import *
from math import pi
from math import atan2
from math import radians, degrees
#from math import deg2rad
theta  = atan2(0.128,0.024)
print(theta)
print (radians(theta))
# d, a, alpha, theta
dh_params2 = np.array([[0.077,0,pi/2,0],
                        [0,0.130,0,theta],
                        [0,0.124,0,-theta],
                        [0,0.126,0,0]])
robot = RobotSerial(dh_params2)

robot.show()
