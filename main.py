import rospy
import numpy as np 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from controller import Controller
from corner_examiner import CornerExaminer

def solve():
    controller = Controller()

    turning_around = False

    while not rospy.is_shutdown():
        if not turning_around:
            if controller.get_forward_distance() < 0.7:
                turning_around = True
            else:
                controller.forward()

        else:
            print()
            #controller.turn_round()

solve()