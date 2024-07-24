#!/usr/bin/python3
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import StatusText, HomePosition
from sensor_msgs.msg import NavSatFix
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import PointCloud2
from colours import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int32, Bool
import numpy as np

# This wrapper handles all the sensor data used by the State Machines
class DroneData:

    _instance = None  # Keep instance reference

    # Ensure singleton behaviour
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            rospy.loginfo("DroneData initialized")
            cls._instance = object.__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
    
        self.odometry = None
        self.bbfrontraw = None
        self.bbdownraw = None
        self.stop = False

        self.Cx = 320.5
        self.Cy = 240.5
        self.fx = 381.36246688113556
        self.fy = 381.36246688113556

        self.cam_shape_x = 640
        self.cam_shape_y = 480
        
        self.bridge = CvBridge()

        type_test = rospy.get_param('~type_test', 'complete')

        self.P1 = float(rospy.get_param("state_machine/P1"))

        # Subscribers
        rospy.Subscriber("/mavros/global_position/local", Odometry, self.odometryCallback)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.globalposCallback)
        rospy.Subscriber("/mavros/statustext/recv", StatusText, self.textCallback)
        rospy.Subscriber("/mavros/home_position/home", HomePosition, self.homeCallback)
        rospy.Subscriber('/S500/camera_down/bounding_boxes', BoundingBoxes, self.bbdowncallback)
        rospy.Subscriber('/S500/camera_front/bounding_boxes', BoundingBoxes, self.bbfrontcallback)

    def odometryCallback(self, data):
        self.odometry = data

    def homeCallback(self, data):
        self.home = data

    def globalposCallback(self, data):
        self.globalpos = data

    def bbdowncallback(self, data):
        self.bbdownraw = data

    def bbfrontcallback(self, data):
        self.bbfrontraw = data


    def textCallback(self, data):
        
        split_str = data.text.split(";")

        if split_str[0] == 'START_SM':
            self.operator_msg = "Start SM"
        
        elif split_str[0] == 'STOP_SM':
            self.operator_msg = "Stop SM"

        else:
            pass

    def get_current_yaw(self):
        return euler_from_quaternion([self.odometry.pose.pose.orientation.x, self.odometry.pose.pose.orientation.y, self.odometry.pose.pose.orientation.z, self.odometry.pose.pose.orientation.w])[2]

    def get_bb_from_id(self, id, cam):
        if self.bbfrontraw and cam == "front":

            for i in range(len(self.bbfrontraw.bounding_boxes)):
                if self.bbfrontraw.bounding_boxes[i].id == int(id):
                    
                    return self.bbfrontraw.bounding_boxes[i]
            
            return []
        
        elif self.bbdownraw and cam == "down":
            for i in range(len(self.bbdownraw.bounding_boxes)):
                if self.bbdownraw.bounding_boxes[i].id == int(id):
                    
                    return self.bbdownraw.bounding_boxes[i]
            
            return []
        else:
            return []
    
    def receive_cam_front(self):
        msg = rospy.wait_for_message("S500/camera_front/color/image_raw", Image, timeout=1)
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding = "bgr8")

    def receive_depth_cam_front(self):
        msg = rospy.wait_for_message("S500/camera_front/depth/image_raw", Image, timeout=1)
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding = "32FC1")

    def receive_cam_down(self):
        msg = rospy.wait_for_message("S500/camera_down/color/image_raw", Image, timeout=1)
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding = "bgr8")

if __name__ == "__main__":
    Wrapper = DroneData()
    
    rospy.init_node('Data_Wrapper')