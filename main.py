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
        controller.forward()

        if not np.less(controller.get_shortest_distance(), 0.5).any():
            print(controller.get_distance_left())
            if controller.get_distance_left() < controller.get_distance_right():
                print("go right")
                controller.right()
            elif controller.get_distance_left() > controller.get_distance_right():
                print("go left")
                controller.left()

solve()