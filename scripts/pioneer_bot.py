#!/usr/bin/env python
from __future__ import division
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import tf
import math

class PioneerBot:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.orientation = 0
        self.P = 2
        self.D = 50
        self.first = True

    def odometry_callback(self,data):
        if(self.first):
            self.first = False
        ########################

        ### Add Subscriber callback code here ###

        #########################

    def drive_to_goal(self, goal_position):
        p1 = [self.x,self.y]
        p2 = goal_position

        print("Start: {}".format(p1))
        print("Goal: {}".format(goal_position))

        u = 0.0
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        last_error = 0.0
        error = 0.0
        diff_error = 0.0
        sum_error = 0.0

        r = rospy.Rate(25) # 25hz
        while(u < 1):
            r.sleep()
            ########################

            ### Add PD code here####

            ########################
            self.move(steer, 0.5)

        print("u: {}".format(u))
        print("cte: {}".format(error))
        print("position: {}, {}".format(goal_position[0],goal_position[1]))
        self.stop()

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
    rospy.init_node('pioneer_bot', anonymous=True)
    robot = PioneerBot()

    while(robot.first):
        pass

    #stops node from exiting
    rospy.spin()

if __name__ == '__main__':
    main()
