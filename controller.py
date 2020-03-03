import rospy
import numpy as np 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

class Controller:
    def __init__(self):
        rospy.init_node("controller")

        #self.corner_examinder_subscriber = rospy.Subscriber("/corner_examiner", String, self.examiner_callback)
        rate = rospy.Rate(10)
        self.corner_examinder_publisher = rospy.Publisher("/corner_examiner", Int32, queue_size=1)
        self.corner_examinder_publisher.publish(3)
        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.twist_publisher = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

    def callback(self, laser_data):
        twist = Twist()
        twist.linear.x = 0.2

        left_third = np.array(laser_data.ranges)[ : len(laser_data.ranges) / 3]
        middle_third = np.array(laser_data.ranges)[len(laser_data.ranges) / 3 : 2 * len(laser_data.ranges) / 3]
        right_third = np.array(laser_data.ranges)[2 * len(laser_data.ranges) / 3 : ]

        # if np.less(np.array(laser_data.ranges), 0.5).any():
        #     twist.linear.x = 0
        #     twist.angular.z = -1
        if np.less(np.array(laser_data.ranges), 0.7).any():
            print("examining that corner..")
            self.corner_examinder_publisher.publish("Examine that corner")
        self.twist_publisher.publish(twist)

    def examiner_callback(self, data):
        pass

    def run(self):
        rospy.spin()

c = Controller()
c.run()