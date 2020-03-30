import rospy
import numpy as np

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from cv2 import imshow, inRange, destroyAllWindows, waitKey
from cv_bridge import CvBridge

class Controller:
    def __init__(self):
        rospy.init_node("controller")

        # Subscribers are required to read data from the robot's sensors. Publishers are required to 
        # control the robot. Double underscores are used to denote private variables that cannot 
        # be accessed by other classes.
        self.__laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.__laser_callback)
        self.__camera_subscriber = rospy.Subscriber("camera/rgb/image_raw", Image, self.__camera_callback)
        self.__twist_publisher = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

        self.__twist = Twist()

        # Provides a connection to the OpenCV library.
        self.__cv_bridge = CvBridge()

        self.left_laser_sum = 0
        self.right_laser_sum = 0
        self.laser_minimum = 0
        self.laser_maximum = 0
        self.laser_data = []

        self.green_square_found = False
        self.red_square_found = False
        self.blue_square_found = False

        self.blue_mask_left = []
        self.blue_mask_right = []

    def __laser_callback(self, laser_data):
        """ Processes laser data recieved from the robot to calculate the minimum and maximum distances, 
        as well as the sum of the laser values of the left and right halves of the laser array. 
        
        Parameters:
        laser_data (LaserScan): The raw data recieved from the robot's Kinect infrared laser sensor.
        
        """

        laser_data = laser_data.ranges

        # When the program starts there may be no laser values in the incoming data, so the 
        # NaN values are replaced with zeros until the actual is recieved.
        try:
            np.nanargmax(laser_data)
        except:
            laser_data = np.zeros(np.shape(laser_data))

        self.laser_data = laser_data
        
        self.laser_minimum = np.nanmin(laser_data)
        self.laser_maximum = np.nanmax(laser_data)

        self.left_laser_sum = np.nansum(laser_data[:len(laser_data) / 2])
        self.right_laser_sum = np.nansum(laser_data[len(laser_data) / 2:])

    def __camera_callback(self, camera_data):

        """ Receives raw camera data from the robot and uses it to determine whether a green, 
        red, or blue square can be seen. OpenCV is used to aid this. 
        
        Parameters:
        camera_data (Image): Raw data recieved via the Image ROS topic.
        
        """

        cv_image = self.__cv_bridge.imgmsg_to_cv2(camera_data, "bgr8")

        self.__set_green_square_flag(cv_image)
        self.__set_red_square_flag(cv_image)
        self.__set_blue_square_flag(cv_image)

        waitKey(1)

    def __set_green_square_flag(self, cv_image):

        """ Changes the value of the green_square_found flag depending on whether 
        any green pixels can be found in the robot's field of view. 
        
        Parameters:
        cv_image (numpy.ndarray): A 3D array containing BGR pixel data from the robot's camera. 
        
        """

        green_mask = inRange(cv_image[:, [100, 400]], (1, 130, 1), (1, 160, 1))

        if np.in1d(green_mask, 255, assume_unique=True).any():
            self.green_square_found = True
        else:
            self.green_square_found = False
    
    def __set_red_square_flag(self, cv_image):

        """ Changes the value of the red_square_found flag depending on whether 
        any red pixels can be found in the robot's field of view. 
        
        Parameters:
        cv_image (numpy.ndarray): A 3D array containing BGR pixel data from the robot's camera. 
        
        """

        red_mask = inRange(cv_image[:][400:420], (1, 1, 130), (1, 1, 160))

        if np.in1d(red_mask, 255, assume_unique=True).any():
            self.red_square_found = True
        else:
            self.red_square_found = False

    def __set_blue_square_flag(self, cv_image):

        """ Changes the value of the blue_square_found flag depending on whether 
        any blue pixels can be found in the robot's field of view. The left and right halves of
        the mask are stored as seperate public variables.
        
        Parameters:
        cv_image (numpy.ndarray): A 3D array containing BGR pixel data from the robot's camera. 
        
        """

        blue_mask = inRange(cv_image, (120, 20, 20), (130, 30, 30))

        if np.in1d(blue_mask, 255, assume_unique=True).any():
            self.blue_square_found = True
        else:
            self.blue_square_found = False

        self.blue_mask_left = blue_mask[:, :320]
        self.blue_mask_right = blue_mask[:, 320:]
    
    def forwards(self):

        """ Publishes twist data that will be used to make the robot move forward. A slight
        leftward drift is applied to encourage the robot to follow the left-hand wall of the maze. """

        self.__twist.angular.z = 0.05
        self.__twist.linear.x = 0.2
        self.__twist_publisher.publish(self.__twist)

    def backwards(self):

        """ Publishes twist data that will be used to make the robot move backwards. The robot recieves 
        this data on the ./velocity topic. """

        self.__twist.angular.z = 0
        self.__twist.linear.x = -0.1
        self.__twist_publisher.publish(self.__twist)

    def stop(self):

        """ Publishes twist data that will be used to make the robot stop all movement. """

        self.__twist.angular.z = 0
        self.__twist.linear.x = 0
        self.__twist_publisher.publish(self.__twist)

    def turn_left(self):

        """ Publishes twist data that will be used to make the robot turn left with an 
        angular velocity of 0.5 radians per second. """

        self.__twist.angular.z = 0.5
        self.__twist.linear.x = 0
        self.__twist_publisher.publish(self.__twist)

    def turn_right(self):

        """ Publishes twist data that will be used to make the robot turn right with an 
        angular velocity of 0.5 radians per second. """

        self.__twist.angular.z = -0.5
        self.__twist.linear.x = 0
        self.__twist_publisher.publish(self.__twist)

    def drift_left(self):

        """ Publishes twist data that will be used to navigate the robot forwards with a slight 
        leftwards drift of 0.25 r/s. """

        self.__twist.linear.x = 0.2
        self.__twist.angular.z = 0.25
        self.__twist_publisher.publish(self.__twist)

    def drift_right(self):

        """ Publishes twist data that will be used to navigate the robot forwards with a slight 
        rightwards drift of 0.25 r/s. """

        self.__twist.linear.x = 0.2
        self.__twist.angular.z = -0.25
        self.__twist_publisher.publish(self.__twist)