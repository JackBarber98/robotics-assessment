import rospy
import math
import numpy as np 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from controller import Controller

def solve():
    controller = Controller()

    following_wall = False
    door_detected = False
    turning_left = False

    while not rospy.is_shutdown():
        if controller.laser_minimum < 0.5 and not door_detected:
            if not door_detected:
                following_wall = True
                controller.stop()
                if controller.left_laser_sum >= controller.right_laser_sum or turning_left:
                    controller.turn_right()
                    turning_left = True
                elif not turning_left:
                    controller.turn_left()
        else: 
            turning_left = False
            if following_wall:
                if controller.laser_data[0] >= 1.2:
                    door_detected = True
                    following_wall = False
            if door_detected:
                if controller.laser_minimum < 0.6:
                    controller.stop()
                    door_detected = False
                else:
                    controller.drift_right()
            else:
                controller.forwards()
        rospy.sleep(0.5)

solve()