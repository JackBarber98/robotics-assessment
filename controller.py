import rospy
import numpy as np 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# main statement to control individual nodes. Use class members to access variables

class Controller:
    def __init__(self):
        rospy.init_node("controller")
        self.corner_examinder_publisher = rospy.Publisher("/corner_examiner", String, queue_size=1)

        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.twist_publisher = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

        self.left_third = []
        self.middle_third = []
        self.right_third = []
        self.laser_data = []

        self.odom = []

    def laser_callback(self, laser_data):
        self.laser_data = laser_data
        self.left_third = np.array(laser_data.ranges)[  : len(laser_data.ranges) / 3]
        self.middle_third = np.array(laser_data.ranges)[len(laser_data.ranges) / 3 : 2 * len(laser_data.ranges) / 3]
        self.right_third = np.array(laser_data.ranges)[2 * len(laser_data.ranges) / 3 : ]

    def odom_callback(self, odom_data):
        self.odom = odom_data.pose.pose.orientation

    def get_forward_distance(self):
        return np.less(self.middle_third, 0.5).any()

    def get_distance_right(self):
        return np.nanmean(self.right_third)

    def get_distance_left(self):
        return np.nanmean(self.left_third)

    def forward(self):
        twist = Twist()
        twist.linear.x = 0.1
        self.twist_publisher.publish(twist)

    def get_shortest_distance(self):
        return self.laser_data

    def turn_round(self):
        twist = Twist()
        twist.angular.z = 5
        self.twist_publisher.publish(twist)

    def left(self):
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = -1
        self.twist_publisher.publish(twist)
    
    def right(self):
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 1
        self.twist_publisher.publish(twist)

    def run(self):
        rospy.spin()