#!/usr/bin/python3
import rospy
import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geographic_msgs.msg import GeoPoseStamped
from tf.transformations import quaternion_from_euler
from Data_Wrapper import DroneData
from mavros_msgs.srv import CommandLong, CommandLongRequest
from std_msgs.msg import Bool, Float64, Float32
from colours import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from skimage import transform
#from sklearn.cluster import DBSCAN
from scipy.stats import mode
import cv2
import math

class Control_Wrapper():

    _instance = None  # Keep instance reference
    
    # Ensure singleton behaviour
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            rospy.loginfo("DroneData initialized")
            cls._instance = object.__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):

        #DroneData.__init__(self)
        
        # GPS Coordinates
        self._GLOBAL_position_stp_msg = GeoPoseStamped()
        self._pub_GLOBAL_position_stp = rospy.Publisher("/mavros/setpoint_position/global", GeoPoseStamped, queue_size=1)

        # LOCAL_NED
        self._LOCAL_position_stp_msg = PoseStamped()
        self._pub_LOCAL_position_stp = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)

        # BODY_NED
        self._MAV_velocity_stp_msg = Twist()
        self._pub_MAV_velocity_stp = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        
        self._pub_open_close_ball = rospy.Publisher("/ball/joint1_position_controller/command", Float64, queue_size=1)
        self._pub_release_ball = rospy.Publisher("/icts_uav/uav_magnet/gain", Float32, queue_size=1)

        self._pub_enable_frontal_NN = rospy.Publisher("/icts/activate_frontal_nn", Bool, queue_size=1)
        self._pub_enable_down_NN = rospy.Publisher("/icts/activate_down_nn", Bool, queue_size=1)

        self.ns = rospy.get_namespace()

        self.command_client = rospy.ServiceProxy(
            name="{}mavros/cmd/command".format(self.ns), service_class=CommandLong
        )

    def publish_global_pos(self):
        self._pub_GLOBAL_position_stp.publish(self._GLOBAL_position_stp_msg)

    def publish_local_pos(self):
        self._pub_LOCAL_position_stp.publish(self._LOCAL_position_stp_msg)

    def publish_vel(self):
        self._pub_MAV_velocity_stp.publish(self._MAV_velocity_stp_msg)

    def activate_frontal_NN(self):
        msg = Bool()
        msg.data = True

        rospy.loginfo(CGREEN2 + "Rede Neural frontal ativada" + CEND)
        self._pub_enable_frontal_NN.publish(msg)

    def activate_down_NN(self):
        msg = Bool()
        msg.data = True

        rospy.loginfo(CGREEN2 + "Rede Neural inferior ativada" + CEND)
        self._pub_enable_down_NN.publish(msg)

    def deactivate_frontal_NN(self):
        msg = Bool()
        msg.data = False

        rospy.loginfo(CGREEN2 + "Rede Neural frontal desativada" + CEND)
        self._pub_enable_frontal_NN.publish(msg)

    def deactivate_down_NN(self):
        msg = Bool()
        msg.data = False

        rospy.loginfo(CGREEN2 + "Rede Neural inferior desativada" + CEND)
        self._pub_enable_down_NN.publish(msg)

    def SetpointGlobalPos(self, x, y, z, yaw):
        self._GLOBAL_position_stp_msg.pose.position.latitude = x
        self._GLOBAL_position_stp_msg.pose.position.longitude = y
        self._GLOBAL_position_stp_msg.pose.position.altitude = z

        quaternion = quaternion_from_euler(0,0,float(yaw))

        self._GLOBAL_position_stp_msg.pose.orientation.x = quaternion[0]
        self._GLOBAL_position_stp_msg.pose.orientation.y = quaternion[1]
        self._GLOBAL_position_stp_msg.pose.orientation.z = quaternion[2]
        self._GLOBAL_position_stp_msg.pose.orientation.w = quaternion[3]

    def SetpointLocalPos(self, x, y, z, yaw):
        self._LOCAL_position_stp_msg.pose.position.x = x
        self._LOCAL_position_stp_msg.pose.position.y = y
        self._LOCAL_position_stp_msg.pose.position.z = z

        quaternion = quaternion_from_euler(0,0,float(yaw))

        self._LOCAL_position_stp_msg.pose.orientation.x = quaternion[0]
        self._LOCAL_position_stp_msg.pose.orientation.y = quaternion[1]
        self._LOCAL_position_stp_msg.pose.orientation.z = quaternion[2]
        self._LOCAL_position_stp_msg.pose.orientation.w = quaternion[3]

    def SetpointLinearVel(self, x, y, z):
        self._MAV_velocity_stp_msg.linear.x = x
        self._MAV_velocity_stp_msg.linear.y = y
        self._MAV_velocity_stp_msg.linear.z = z

    def SetpointAngularVel(self, roll, pitch, yaw):
        self._MAV_velocity_stp_msg.angular.x = roll
        self._MAV_velocity_stp_msg.angular.y = pitch
        self._MAV_velocity_stp_msg.angular.z = yaw

    def CheckPositionReached(self, init_position, final_position):

        dx = abs(
            init_position.x - final_position.x
        )
        dy = abs(
            init_position.y - final_position.y
        )
        dz = abs(
            init_position.z - final_position.z
        )
        
        dMag = math.sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

        # TO-DO Acrescentar verificação de yaw

        if dMag < rospy.get_param("state_machine/P2"):
            return True
        else:
            return False

    def CheckGlobalPositionReached(self, init_position, final_position):

        if self.distance_btw_coordinates(init_position.latitude, init_position.longitude, final_position.latitude, final_position.longitude) < rospy.get_param("state_machine/P2"):
            return True
        else:
            return False

    def calculate_angle(self, a, b):

        delta_x = b[0] - a[0]
        delta_y = b[1] - a[1]

        angle_radians = math.atan2(delta_y, delta_x)

        return angle_radians

    def distance_btw_coordinates(self, lat1, lon1, lat2, lon2):
        # Converte graus para radianos
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Diferenças entre as coordenadas
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        # Fórmula de Haversine
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        # Raio da Terra em metros (6371 km * 1000 m/km)
        r = 6371000
        
        # Distância em metros
        distance = c * r
        
        return distance
        
    def calculate_vector_given_down_bb(self, cam_center_point, bb_center_point):

        angle = self.get_angle_btw_2d_points(cam_center_point, bb_center_point)

        x_component = math.cos(angle)
        y_component = math.sin(angle)
    
        return x_component, y_component
    
    def get_angle_btw_2d_points(self, point_1, point_2):

        point_1 = np.array(point_1).astype(dtype = 'float32')
        point_2 = np.array(point_2).astype(dtype = 'float32')

        # Line vector
        line_vector = point_1 - point_2

        # Angulation in rad
        _, angles = cv2.cartToPolar(line_vector[..., 0], line_vector[..., 1])
        
        return angles

    def get_distance_btw_2d_points(self, point_1, point_2):
        
        x1, y1 = point_1
        x2, y2 = point_2

        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        return distance