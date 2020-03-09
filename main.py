import rospy
import math
import numpy as np 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from controller import Controller

def solve():
    controller = Controller()

    turn_left = False

    while not rospy.is_shutdown():
        if not controller.is_close_to_wall():
            controller.forward()
        else:
            controller.left()

solve()