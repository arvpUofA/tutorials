#!/usr/bin/env python

# author : Noni
# created on : May 18th
# modified from : https://raw.githubusercontent.com/eacousineau/executive_smach_tutorials/hydro-devel/smach_tutorials/examples/user_data2.py

import roslib#; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseGoal
import math
from turtlebot import Turtlebot

# helper functions
def close_to_goal(goal, position):
    x, y = goal.target_pose.pose.position.x, goal.target_pose.pose.position.y
    d = math.sqrt((x - position[0])**2 + (y - position[1])**2)
    print "Distance to goal:", d
    return d < 0.5

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.header.stamp = rospy.Time.now()
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

# define state Searching
class Searching(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['found target','not found'])
        self.robot = Turtlebot()
        self.pickup_location = [(-0.29, -4.64, 0.0), (0.0, 0.0, -0.68, 0.74)]
        self.goal = goal_pose(self.pickup_location)

    def execute(self, userdata):
        rospy.loginfo('Executing state Searching')
# Re-define state Going to a new place
########################## BEGINNING OF YOUR CODE ###############################
        if close_to_goal(self.goal, self.robot.position):
            return 'found target'
        else:
            self.robot.drive_to_goal(self.pickup_location)
            return 'not found'
############################# END OF YOUR CODE ##################################

# define state Grabbing
class Grabbing(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['finished grabbing', 'keep grabbing'])
        self.robot = Turtlebot()

    def execute(self, userdata):
        rospy.loginfo('Executing state Grabbing')
        rospy.loginfo('Counter = %f'%self.robot.grabbing_count)
        if self.robot.grabbing_count >= 5:
            rospy.sleep(1)
            return 'finished grabbing'
        else:
            self.robot.loading()
            return 'keep grabbing'

# Assignment from last time
class ToNewLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['finished turning', 'keep turning'])
        self.robot = Turtlebot()
        self.drop_location = [( 1.02, -3.10, 0.0), (0.0, 0.0, -0.94, 0.33)]
        self.goal = goal_pose(self.drop_location)

    def execute(self, userdata):
        rospy.loginfo('Executing state ToNewLocation')
# Re-define state Going to a new place
########################## BEGINNING OF YOUR CODE ###############################
        if close_to_goal(self.goal, self.robot.position):
            return 'found target'
        else:
            self.robot.drive_to_goal(self.drop_location)
            return 'not found'
############################# END OF YOUR CODE ##################################

# define state Dropping
class Dropping(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['finished dropping', 'keep dropping'])
        self.robot = Turtlebot()

    def execute(self, userdata):
        rospy.loginfo('Executing state Dropping')
        rospy.loginfo('Counter = %f'%self.robot.grabbing_count)
        if self.robot.grabbing_count == 0:
            return 'finished dropping'
        else:
            self.robot.unloading()
            return 'keep dropping'


def main():
    rospy.init_node('turtlebot_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['initial'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Searching', Searching(),
                               transitions={'found target':'Grabbing',
                                            'not found':'Searching'})
        smach.StateMachine.add('Grabbing', Grabbing(),
                               transitions={'finished grabbing':'ToNewLocation',
                                            'keep grabbing':'Grabbing'})
        smach.StateMachine.add('ToNewLocation', ToNewLocation(),
                               transitions={'finished turning':'Dropping',
                                            'keep turning':'ToNewLocation'})
        smach.StateMachine.add('Dropping', Dropping(),
                                transitions={'finished dropping':'Searching',
                                             'keep dropping':'Dropping'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
