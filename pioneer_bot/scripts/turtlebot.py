#!/usr/bin/env python

# June 2017 ROS Edmonton Tutorial

from __future__ import division

# ROS libs
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib
# non-ROS libs
import math
import tf
import numpy as np

class Turtlebot:

    def __init__(self):

        self.velocity_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=1)
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)
        self.bridge = CvBridge()
        self.sound = SoundClient()

        self.x = 0
        self.y = 0
        self.z = 0
        self.orientation = 0
        self.P = 2
        self.D = 25
        self.first = True
        self.done = False

        self.grabbing_count = 0
        self.target_reached = False

        self.position = [0,0]
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def amcl_cb(self, message):
        self.position[0] = pose.pose.pose.position.x
        self.position[1] = pose.pose.pose.position.y

    def loading(self):
        if self.grabbing_count <= 6:
            self.grabbing_count = self.grabbing_count + 1

    def unloading(self):
        if self.grabbing_count > 1:
            self.grabbing_count = self.grabbing_count - 1

    def drive_to_goal(self, goal):
        self.client.wait_for_server()

        self.client.send_goal(goal)
        self.client.wait_for_result()
        GOAL_REACHED = True;

    def move(self, steer, speed):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = steer
        self.velocity_pub.publish(msg)

    def stop(self):
        msg = Twist()
        self.velocity_pub.publish(msg)

def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('turtle_bot', anonymous=True)
    robot = Turtlebot()

    #wait for first odometry callback
    while(robot.first):
        pass

    #stops node from exiting
    rospy.spin()

if __name__ == '__main__':
    main()
