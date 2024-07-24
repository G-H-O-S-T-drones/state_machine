#!/usr/bin/python3
import smach
from States_AwaitingAutonomous import *

awaitingAutonomous = smach.StateMachine(outcomes=['Done', 'Exit_SM'])

with awaitingAutonomous:
    # Declaring state
    smach.StateMachine.add('AwaitingAutonomous', AwaitingAutonomous(),
                           transitions={'Waiting_For_Autonomous': 'AwaitingAutonomous',
                                        'Received': 'Done',
                                        'Exit_SM': 'Exit_SM'})


if __name__ == '__main__':
    awaitingAutonomous.execute()