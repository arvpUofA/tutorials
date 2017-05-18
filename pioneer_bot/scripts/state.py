#!/usr/bin/env python

# author : Noni
# created on : May 18th 
# modified from : https://raw.githubusercontent.com/eacousineau/executive_smach_tutorials/hydro-devel/smach_tutorials/examples/user_data2.py

import roslib#; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

# define state Searching
class Searching(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['found target','not found'],
                             input_keys=['target_counter_in'],
                             output_keys=['target_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Searching')
        if userdata.target_counter_in < 10:
            userdata.target_counter_out = userdata.target_counter_in + 1
            return 'not found'
        else:
            return 'found target'


# define state Grabbing
class Grabbing(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['finished grabbing', 'keep grabbing'],
                             input_keys=['grabbing_counter_in'],
                             output_keys=['grabbing_counter_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Grabbing')
        rospy.loginfo('Counter = %f'%userdata.grabbing_counter_in)
        if userdata.grabbing_counter_in < 1:       
            return 'finished grabbing'
        else:
            userdata.grabbing_counter_out = userdata.grabbing_counter_in - 1 
            return 'keep grabbing'

# define state Going to a new place
########################## BEGINNING OF YOUR CODE ###############################


############################# END OF YOUR CODE ##################################

# define state Dropping
class Dropping(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['dropped', 'still holding'],
                             input_keys=['dropping_counter_in'],
                             output_keys=['dropping_counter_out'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Dropping')
        rospy.loginfo('Counter = %f'%userdata.dropping_counter_in)
        if userdata.dropping_counter_in == 3:
            return 'dropped'
        else:
            userdata.dropping_counter_out = userdata.dropping_counter_in + 1
            return 'still holding'


def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['initial'])
    sm.userdata.sm_counter = 0
    sm.userdata.object_counter = 3

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Searching', Searching(), 
                               transitions={'found target':'Grabbing', 
                                            'not found':'Searching'},
                               remapping={'target_counter_in':'sm_counter', 
                                          'target_counter_out':'sm_counter'})
        smach.StateMachine.add('Grabbing', Grabbing(), 
                               transitions={'finished grabbing':'Dropping',
                                            'keep grabbing':'Grabbing'},
                               remapping={'grabbing_counter_in':'object_counter',
                                          'grabbing_counter_out':'object_counter'})
        smach.StateMachine.add('Dropping', Dropping(),
                                transitions={'dropped':'Searching',
                                             'still holding':'Dropping'},
                                remapping={'dropping_counter_in':'object_counter',
                                           'dropping_counter_out':'object_counter'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
