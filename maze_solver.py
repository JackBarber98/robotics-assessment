import numpy as np 
import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from controller import Controller

class MazeSolver():

    def __init__(self):

        self.__controller = Controller()

        self.__following_wall = False
        self.__gap_detected = False
        self.__turning_left = False

        self.__avoiding_trap = False
        self.__moving_to_exit = False
        self.__moving_to_waypoint = False

        # Creates an empty plot with a maximum y-value of 6.
        plt.ion()
        self.__fig = plt.figure()
        self.__ax = self.__fig.add_subplot(111)
        plt.ylim(0, 6)

    def run(self):

        """ The main control loop for managing the inhibition and exhibition of the robot's behaviours 
        for navigating the maze. """

        while not rospy.is_shutdown():

            # If a trap has been detected, the avoid trap behaviour should be started.
            if self.__avoiding_trap and not self.__moving_to_exit and not self.__moving_to_waypoint:
                self.__avoid_trap()

            # If the exit is found, the robot should move towards it.
            elif self.__moving_to_exit and not self.__avoiding_trap and not self.__moving_to_waypoint:
                self.__go_to_exit()
            elif self.__moving_to_waypoint and not self.__moving_to_exit and not self.__moving_to_exit:
                self.__go_to_waypoint()

            # During "normal" behaviour when the flags indicating traps, exits, and waypoints are False, 
            # the robot should attempt to follow the wall. This is the default behaviour.
            else:
                self.__follow_wall()
                self.__identify_squares()

            self.__plot_laser_data()

            rospy.sleep(0.5)
        plt.show(block=True)

    def __plot_laser_data(self):

        """ Plots the Kinect / laser scanner data published on std_msgs/LaserScan, updating 
        in real-time. This isn't required to make the robot navigate the maze - it's just 
        interesting to look at! """

        self.__ax.clear()
        self.__ax.set_title("Kinect Distances")
        self.__ax.set_xlabel("Laser Index")
        self.__ax.set_ylabel("Distance (meters)")
        self.__ax.plot(self.__controller.laser_data)
        self.__fig.canvas.draw()

    def __move_to_most_open_space(self):

        """ Used to make the robot move towards the most open space when turning away from a wall. """

        # The "turning left" variable is used to prevent the robot getting stuck in corners.
        if self.__controller.left_laser_sum >= self.__controller.right_laser_sum or self.__turning_left:
            self.__controller.turn_right()
            self.__turning_left = True
        else:
            self.__controller.turn_left()

    def __follow_wall(self):

        """ Defines a behaviour that allows the robot to identify, move towards, and follow the maze 
        wall. If the robot's Kinect detects something less than 0.5m away, it turns to be parallel to 
        the wall and exits this behaviour. If a potential gap in the maze is found, the robot moves 
        forwards with a leftwards bias to attempt to move through / close to it. """

        if self.__controller.laser_minimum < 0.5 and not self.__gap_detected:
            if not self.__gap_detected:
                self.__following_wall = True
                self.__controller.stop()

                self.__move_to_most_open_space()
        else: 
            self.__turning_left = False
            if self.__following_wall:
                if self.__controller.laser_data[0] >= 1.2:
                    self.__gap_detected = True
                    self.__following_wall = False

            if self.__gap_detected:
                if self.__controller.laser_minimum < 0.6:
                    self.__controller.stop()
                    self.__gap_detected = False
                else:
                    self.__controller.drift_left()
            else:
                self.__controller.forwards()

    def __identify_squares(self):

        """ Checks whether the robot has seen any traps, waypoints, or the exit and adjusts the
        appropriate flags as required. """

        if self.__controller.red_square_found:
            self.__avoiding_trap = True
            
        if self.__controller.green_square_found:
            self.__moving_to_exit = True

        if self.__controller.blue_square_found:
            self.__moving_to_waypoint = True

    def __avoid_trap(self):

        """ A behaviour whereby the robot turns away from a trap / red square until it is 
        facing away from both the trap and any walls. """

        # The robot is designed to turn towards the most "open" space it can see. The direction of 
        # movement cannot be changed once it starts to prevent the robot getting stuck. 
        if self.__controller.left_laser_sum >= self.__controller.right_laser_sum or self.__turning_left:
            self.__controller.turn_left()
            self.__turning_left = True
        else:
            self.__controller.turn_right()

        if self.__controller.laser_minimum > 0.5 and not self.__controller.red_square_found:
            self.__avoiding_trap = False
            self.__turning_left = False

    def __go_to_exit(self):

        """ The robot will move towards the exit when sighted until a wall is detected in front 
        of it; at this point the robot stops moving and navigation is complete. """

        if self.__controller.green_square_found:
            self.__controller.forwards()

        if self.__controller.laser_minimum < 0.5:
            self.__moving_to_exit = False

        if not self.__controller.green_square_found:
            print("I FOUND THE EXIT")

    def __go_to_waypoint(self):

        """ This behaviour makes the robot navigate towards blue waypoints by moving forwards with either 
        a left or right bias depending on which side of the input image contains the most blue pixels. Once 
        the waypoint is less than 0.5m away, the robot turns to face the most open space parallel 
        to the wall and exits the behaviour. """

        if self.__controller.blue_square_found:
            left_blue_pixel_count = np.count_nonzero(self.__controller.blue_mask_left==255)
            right_blue_pixel_count = np.count_nonzero(self.__controller.blue_mask_right==255)

            if left_blue_pixel_count >= right_blue_pixel_count:
                self.__controller.drift_left()
            else:
                self.__controller.drift_right()
        else:
           self.__moving_to_waypoint = False

        if self.__controller.laser_minimum < 0.5:
            self.__controller.stop()
            self.__move_to_most_open_space()

            self.__moving_to_waypoint = False

if __name__ == "__main__":
    maze_solver = MazeSolver()
    maze_solver.run()