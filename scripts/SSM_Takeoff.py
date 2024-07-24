#!/usr/bin/python3
import smach
from States_Takeoff import *

takeoff = smach.StateMachine(outcomes=['Done'])

with takeoff:
    # Declaring state
    smach.StateMachine.add('Takeoff', Takeoff(),
                           transitions={'Waiting_To_Reach_Waypoint': 'Waiting_To_Reach_Waypoint'})

    smach.StateMachine.add('Waiting_To_Reach_Waypoint', GoToWaypoint(),
                           transitions={'Waiting_To_Reach_Waypoint': 'Waiting_To_Reach_Waypoint',
                                        'Reached': 'Done'})
    
if __name__ == '__main__':
    takeoff.execute()