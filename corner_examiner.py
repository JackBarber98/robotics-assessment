import rospy
import numpy as np 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class CornerExaminer:
    def __init__(self):
        rospy.init_node("corner_examiner")

        self.controller_subscriber = rospy.Subscriber("/controller", String, self.callback)
        #self.controller_publisher = rospy.Publisher("/controller", String, queue_size=1)

    def callback(self, data):
        print(data)

    def run(self):
        rospy.spin()

ce = CornerExaminer()
ce.run()