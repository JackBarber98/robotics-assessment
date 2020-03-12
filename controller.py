import rospy
import numpy as np 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class Controller:
    def __init__(self):
        rospy.init_node("controller")

        self.__laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.__twist_publisher = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
        
        self.__twist = Twist()

        self.left_laser_sum = 0
        self.right_laser_sum = 0
        self.laser_minimum = 0
        self.laser_maximum = 0
        self.laser_data = []

        self.wheel_radius = .06
        self.robot_radius = .2

    def laser_callback(self, laser_data):
        laser_data = laser_data.ranges

        try:
            np.nanargmax(laser_data)
        except:
            laser_data = np.zeros(np.shape(laser_data))
        
        self.left_laser_sum = 0 
        self.right_laser_sum = 0
        self.laser_minimum = np.nanmin(laser_data)
        self.laser_maximum = np.nanmax(laser_data)

        self.left_laser_sum = np.nansum(laser_data[:len(laser_data) / 2])
        self.right_laser_sum = np.nansum(laser_data[len(laser_data) / 2:])

        self.laser_data = laser_data
    
    def forwards(self):
        self.__twist.angular.z = 0
        self.__twist.linear.x = 0.2
        self.__twist_publisher.publish(self.__twist)

    def backwards(self):
        self.__twist.angular.z = 0.1
        self.__twist.linear.x = -0.2
        self.__twist_publisher.publish(self.__twist)

    def stop(self):
        self.__twist.angular.z = 0
        self.__twist.linear.x = 0
        self.__twist_publisher.publish(self.__twist)

    def turn_left(self):
        self.__twist.angular.z = 0.5
        self.__twist.linear.x = 0
        self.__twist_publisher.publish(self.__twist)

    def turn_right(self):
        self.__twist.angular.z = -0.5
        self.__twist.linear.x = 0
        self.__twist_publisher.publish(self.__twist)

    def drift_right(self):
        self.__twist.linear.x = 0.2
        self.__twist.angular.z = 0.25
        self.__twist_publisher.publish(self.__twist)

    def run(self):
        rospy.spin()