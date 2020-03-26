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

    avoiding_trap = False
    going_to_exit = False
    moving_to_blue = False

    while not rospy.is_shutdown():
        if not avoiding_trap and not going_to_exit and not moving_to_blue:
            if controller.laser_minimum < 0.5 and not door_detected:
                if not door_detected:
                    following_wall = True
                    controller.stop()
                    if controller.left_laser_sum >= controller.right_laser_sum or turning_left:
                        controller.turn_right()
                        turning_left = True
                    else:
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
                        controller.drift_left()
                else:
                    controller.forwards()
            
            if controller.red_square_found:
                avoiding_trap = True
            if controller.green_square_found:
                going_to_exit = True
            if controller.blue_square_found:
                moving_to_blue = True
        elif avoiding_trap and not going_to_exit and not moving_to_blue:
            print("AVOIDING TRAP")
            if controller.left_laser_sum >= controller.right_laser_sum or turning_left:
                controller.turn_left_backwards()
                turning_left = True
            else:
                controller.turn_right_backwards()
            if controller.laser_minimum > 0.5 and not controller.red_square_found:
                avoiding_trap = False
                turning_left = False
                print("I AVOIDED THE TRAP!")
        elif going_to_exit and not avoiding_trap and not moving_to_blue:
            if controller.green_square_found:
                controller.forwards()
            if controller.laser_minimum < 0.5:
                going_to_exit = False
            if not controller.green_square_found:
                print("I FOUND THE EXIT")
        else:
            if controller.blue_square_found:
                left_blue_pixel_count = np.count_nonzero(controller.blue_mask_left==255)
                right_blue_pixel_count = np.count_nonzero(controller.blue_mask_right==255)

                if left_blue_pixel_count >= right_blue_pixel_count:
                    controller.drift_left()
                else:
                    controller.drift_right()
            else:
                moving_to_blue = False

            if controller.laser_minimum < 0.5:
                controller.stop()
                if controller.left_laser_sum >= controller.right_laser_sum or turning_left:
                    controller.turn_right()
                    turning_left = True
                elif not turning_left:
                    controller.turn_left()
                moving_to_blue = False

        rospy.sleep(0.5)

solve()