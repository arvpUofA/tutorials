#!/usr/bin/env python
from __future__ import division
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class PioneerBot:

    def __init__(self):

        self.odometry_sub = rospy.Subscriber("pioneer2dx/odom", Odometry, self.odometry_callback)
        self.velocity_pub = rospy.Publisher("pioneer2dx/cmd_vel", Twist, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/debug_image", Image, queue_size=1)
        self.bridge = CvBridge()
        
        self.x = 0
        self.y = 0
        self.z = 0
        self.orientation = 0
        self.P = 2
        self.D = 25
        self.first = True
        self.done = False

    def odometry_callback(self,data):
        if(self.first):
            self.first = False
        orientation = data.pose.pose.orientation
        position = data.pose.pose.position

        self.x = position.x
        self.y = position.y

        quaternion = (orientation.x,orientation.y,orientation.z,orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.orientation = euler[2] if euler[2] > 0 else euler[2] + 2*math.pi

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
            rx =  self.x - p1[0]
            ry =  self.y - p1[1]

            u  = (rx * dx + ry * dy) / (dx * dx + dy * dy)
            error = (ry * dx - rx * dy) / math.sqrt(dx * dx + dy * dy)

            diff_error = error - last_error
            last_error = error

            steer = - error * self.P - diff_error * self.D
            # Note with Gazebo 2, a bug in the ROS plugin means you have to pass the negative of the steering commands
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
    
    def image_callback(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
        except CvBridgeError as e:
            print(e)

        cv_image = self.track_ball(cv_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
        except CvBridgeError as e:
            print(e)
    
    def track_ball(self, image):
        (rows,cols,channels) = image.shape
        
        ###Code 1 goes here

        ####

        ####Code 2 goes here
        
        ####
        
        ####Code 3 goes here
        
        ####
        
        return image
         


def main():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pioneer_bot', anonymous=True)
    robot = PioneerBot()

    #wait for first odometry callback
    while(robot.first):
        pass

    #stops node from exiting
    rospy.spin()

if __name__ == '__main__':
    main()
