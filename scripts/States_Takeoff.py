#!/usr/bin/python3
import rospy
import smach
from geometry_msgs.msg import Point
from Control_Wrapper import Control_Wrapper
from Data_Wrapper import DroneData

class Takeoff(smach.State, DroneData):

    def __init__(self):
        smach.State.__init__(self, outcomes=["Waiting_To_Reach_Waypoint", "Exit_SM"])
        DroneData.__init__(self)
        self.control = Control_Wrapper()

    def execute(self, userdata):

        if self.operator_msg == "Stop SM":
            return "Exit_SM"
        
        self.control.SetpointMavPos(0, 0, self.P1, 0)

        self.control.publish_mav_pos()
        
        return 'Waiting_To_Reach_Waypoint'
    
class GoToWaypoint(smach.State, DroneData):

    def __init__(self):
        smach.State.__init__(self, outcomes=["Waiting_To_Reach_Waypoint", "Reached", "Exit_SM"])
        DroneData.__init__(self)
        self.control = Control_Wrapper()

        self.first_iteration = True
        self.waypoint = Point()

    def execute(self, userdata):

        if self.operator_msg == "Stop SM":
            return "Exit_SM"
        
        curr_pos = self.odometry.pose.pose.position

        if self.first_iteration:
            self.waypoint.x = self.odometry.pose.pose.position.x
            self.waypoint.y = self.odometry.pose.pose.position.y
            self.waypoint.z = self.P1

        if self.control.CheckPositionReached(curr_pos, self.waypoint) == False:
            return 'Waiting_To_Reach_Waypoint'
        else:
            return 'Reached'