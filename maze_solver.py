import numpy as np 

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from controller import Controller

class MazeSolver():

    def __init__(self):

        self.__controller = Controller()

        self.__following_wall = False
        self.__door_detected = False
        self.__turning_left = False

        self.__avoiding_trap = False
        self.__going_to_exit = False
        self.__moving_to_blue = False

    def run(self):
        while not rospy.is_shutdown():
            if not self.__avoiding_trap and not self.__going_to_exit and not self.__moving_to_blue:
                self.follow_wall()
                self.identify_squares()

            elif self.__avoiding_trap and not self.__going_to_exit and not self.__moving_to_blue:
                self.avoid_trap()

            elif self.__going_to_exit and not self.__avoiding_trap and not self.__moving_to_blue:
                self.go_to_exit()
            else:
                self.go_to_waypoint()

            rospy.sleep(0.5)

    def follow_wall(self):
        if self.__controller.laser_minimum < 0.5 and not self.__door_detected:
            if not self.__door_detected:
                self.__following_wall = True
                self.__controller.stop()
                if self.__controller.left_laser_sum >= self.__controller.right_laser_sum or self.__turning_left:
                    self.__controller.turn_right()
                    self.__turning_left = True
                else:
                    self.__controller.turn_left()
        else: 
            self.__turning_left = False
            if self.__following_wall:
                if self.__controller.laser_data[0] >= 1.2:
                    self.__door_detected = True
                    self.__following_wall = False
            if self.__door_detected:
                if self.__controller.laser_minimum < 0.6:
                    self.__controller.stop()
                    self.__door_detected = False
                else:
                    self.__controller.drift_left()
            else:
                self.__controller.forwards()

    def identify_squares(self):
        if self.__controller.red_square_found:
            self.__avoiding_trap = True
        if self.__controller.green_square_found:
            self.__going_to_exit = True
        if self.__controller.blue_square_found:
            self.__moving_to_blue = True

    def avoid_trap(self):
        print("AVOIDING TRAP")
        if self.__controller.left_laser_sum >= self.__controller.right_laser_sum or self.__turning_left:
            self.__controller.turn_left()
            self.__turning_left = True
        else:
            self.__controller.turn_right()

        if self.__controller.laser_minimum > 0.5 and not self.__controller.red_square_found:
            self.__avoiding_trap = False
            self.__turning_left = False
            print("I AVOIDED THE TRAP!")

    def go_to_exit(self):
        if self.__controller.green_square_found:
            self.__controller.forwards()
        if self.__controller.laser_minimum < 0.5:
            self.__going_to_exit = False
        if not self.__controller.green_square_found:
            print("I FOUND THE EXIT")

    def go_to_waypoint(self):
        if self.__controller.blue_square_found:
            left_blue_pixel_count = np.count_nonzero(self.__controller.blue_mask_left==255)
            right_blue_pixel_count = np.count_nonzero(self.__controller.blue_mask_right==255)

            if left_blue_pixel_count >= right_blue_pixel_count:
                self.__controller.drift_left()
            else:
                self.__controller.drift_right()
        else:
           self.__moving_to_blue = False

        if self.__controller.laser_minimum < 0.5:
            self.__controller.stop()
            if self.__controller.left_laser_sum >= self.__controller.right_laser_sum or self.__turning_left:
                self.__controller.turn_right()
                self.__turning_left = True
            elif not self.__turning_left:
                self.__controller.turn_left()
            self.__moving_to_blue = False

maze_solver = MazeSolver()
maze_solver.run()