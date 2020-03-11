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

    while not rospy.is_shutdown():
        #try:
        if controller.laser_minimum < 0.5 and not door_detected:
            print("too close...") 
            if not door_detected:
                print("and the door isn't detected")
                following_wall = True
                controller.stop()
                if controller.left_laser_sum >= controller.right_laser_sum:
                    controller.turn_right()
                else:
                    controller.turn_left()
            else:
                print("and the door is detected")
        else: 
            if following_wall:
                if controller.laser_maximum >= 2:
                    door_detected = True
                    following_wall = False
                    print("door detected")
            if door_detected:
                print("laser min: ", controller.laser_minimum)
                if controller.laser_minimum < 0.5:
                    print("gonna crash into wall (drifting)")
                    door_detected = False
                else:
                    print("drifting...")
                    controller.drift_right()
            elif controller.laser_minimum >= 0.5:
                controller.forwards()
        # except:
        #     controller.backwards()
        rospy.sleep(0.5)

solve()