#!/usr/bin/python3
import rospy
import smach

from Data_Wrapper import DroneData
from Control_Wrapper import Control_Wrapper

# Open Loop (State 1 - AwaitingAutonomous)
class AwaitingAutonomous(smach.State, DroneData):

    def __init__(self):
        smach.State.__init__(self, outcomes=["Waiting_For_Autonomous", "Received", "Exit_SM"])
        DroneData.__init__(self)
        self.control = Control_Wrapper()

    def execute(self, userdata):

        if self.operator_msg == "Stop SM":
            return "Exit_SM"
        
        elif self.operator_msg == "Start SM":
            return "Received"

        else:
            return "Waiting_For_Autonomous"