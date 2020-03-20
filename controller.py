import rospy
import numpy as np 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String

from cv2 import namedWindow, cvtColor, imshow, inRange
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey
from cv2 import Canny
from cv_bridge import CvBridge

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

        self.green_square_found = False
        self.red_square_found = False

        self.__cv_bridge = CvBridge()
        self.__camera_subscriber = rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_callback)

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

    def camera_callback(self, camera_data):
        cv_image = self.__cv_bridge.imgmsg_to_cv2(camera_data, "bgr8")
        middle_column = cv_image[:, [300, 400]]

        green_mask = inRange(middle_column, (1, 130, 1), (1, 160, 1))
        red_mask = inRange(middle_column, (1, 1, 130), (1, 1, 160))

        if 255 not in green_mask:
            self.green_square_found = True
        else:
            self.green_square_found = False

        if 255 in red_mask:
            self.red_square_found = True
        else:
            self.red_square_found = False

        waitKey(1)
    
    def forwards(self):
        self.__twist.angular.z = 0.05
        self.__twist.linear.x = 0.2
        self.__twist_publisher.publish(self.__twist)

    def backwards(self):
        self.__twist.angular.z = 0
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

    def turn_backwards(self):
        self.__twist.linear.x = -0.2
        self.__twist.angular.z = 2
        self.__twist_publisher.publish(self.__twist)

    def run(self):
        rospy.spin()