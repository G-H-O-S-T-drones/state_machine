#!/usr/bin/python3
import rospy
import smach

if __name__ == '__main__':
    rospy.init_node('main_statemachine')

# Closed Loop SM
from SSM_AwaitingAutonomous import *
from SSM_Takeoff import *

# Declaring the main state machine
rospy.Rate(1)
rospy.loginfo("Starting Main State Machine...")

mainStateMachine = smach.StateMachine(outcomes=['End'])

type_test = rospy.get_param('~type_test', 'complete')

with mainStateMachine:

    if type_test == "fase_1":

        smach.StateMachine.add('AwaitingAutonomous', awaitingAutonomous,
                            transitions={'Done': 'Takeoff',
                                         'Exit_SM': 'End'})

        smach.StateMachine.add('Takeoff', takeoff, 
                            transitions={'Done': 'End',
                                         'Exit_SM': 'End'})
        
    elif type_test == "fase_2":

        smach.StateMachine.add('AwaitingAutonomous', awaitingAutonomous,
                            transitions={'Done': 'Takeoff',
                                         'Exit_SM': 'End'})

        smach.StateMachine.add('Takeoff', takeoff, 
                            transitions={'Done': 'End',
                                         'Exit_SM': 'End'})
        
    elif type_test == "fase_3":

        smach.StateMachine.add('AwaitingAutonomous', awaitingAutonomous,
                            transitions={'Done': 'Takeoff',
                                         'Exit_SM': 'End'})

        smach.StateMachine.add('Takeoff', takeoff, 
                            transitions={'Done': 'End',
                                         'Exit_SM': 'End'})
    elif type_test == "fase_4":

        smach.StateMachine.add('AwaitingAutonomous', awaitingAutonomous,
                            transitions={'Done': 'Takeoff',
                                         'Exit_SM': 'End'})

        smach.StateMachine.add('Takeoff', takeoff, 
                            transitions={'Done': 'End',
                                         'Exit_SM': 'End'})
    else:
        pass

# Executing main state machine
mainStateMachine.execute()