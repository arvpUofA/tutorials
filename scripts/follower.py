#!/usr/bin/env python
import rospy
import pickle
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sklearn.ensemble import RandomForestClassifier
import numpy as np
import os


class Follower:
    def __init__(self):
        rospy.loginfo('Follower node initialized')
        self.msg = None
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.clf = pickle.load(open(os.path.join(os.path.dirname(__file__), "clf"), "rb"))
        self.clf2 = pickle.load(open(os.path.join(os.path.dirname(__file__), "clf2"), "rb"))
        self.labels = {'30_0': 0, '30_l': 1, '30_r': 2, '45_0': 3, '45_l': 4, '45_r': 5, '15_0': 6, 'empty': 7}
        rospy.loginfo('Tree initialized')
        self.follow()

    def laser_scan(self):
        data_test = []
        data_test_set = []
        self.msg = rospy.wait_for_message("/scan", LaserScan)

        for i in range(70, -2, -1) + range(359, 289, -1):

            if np.nan_to_num(self.msg.ranges[i]) != 0:
                data_test.append(np.nan_to_num(self.msg.ranges[i]))

            elif (i+1) in range(70, -2, -1) + range(359, 289, -1) and (i-1) in range(70, -2, -1) + range(359, 289, -1) and np.nan_to_num(self.msg.ranges[i]) == 0:
                data_test.append((np.nan_to_num(self.msg.ranges[i+1])+np.nan_to_num(self.msg.ranges[i-1]))/2)

            else:
                data_test.append(np.nan_to_num(self.msg.ranges[i]))

        data_test_set.append(data_test)

        x = [x for (x, y) in self.labels.iteritems() if y == self.clf.predict(data_test_set)]

        print(x)

        twist = Twist()
        # Do something according to each position
        if x == ['30_0']:
            twist.linear.x = 0.13
            twist.angular.z = 0.0
        elif x == ['30_l']:
            twist.linear.x = 0.10
            twist.angular.z = 0.4
        elif x == ['30_r']:
            twist.linear.x = 0.10
            twist.angular.z = -0.4
        elif x == ['45_0']:
            twist.linear.x = 0.13
            twist.angular.z = 0.0
        elif x == ['45_l']:
            twist.linear.x = 0.10
            twist.angular.z = 0.3
        elif x == ['45_r']:
            twist.linear.x = 0.10
            twist.angular.z = -0.3
        elif x == ['15_0']:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif x == ['empty']:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)

    def laser_scan_simple(self):
        self.msg = rospy.wait_for_message("/scan", LaserScan)
        range_ahead = self.msg.ranges[0]

        print(range_ahead)

        twist = Twist()
        if 0.3 < range_ahead < 0.7:
            twist.linear.x = 0.13
            twist.angular.z = 0.0

        elif 0 < range_ahead < 0.3:
            twist.linear.x = -0.07
            twist.angular.z = 0.0

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)

    def laser_scan_bigger_range(self):
        self.msg = rospy.wait_for_message("/scan", LaserScan)
        range_ahead = self.msg.ranges[:45] + self.msg.ranges[-45:]

        twist = Twist()
        if 0.4 < sum(range_ahead[30:60]) / 30 < 0.8:
            twist.linear.x = 0.13
            twist.angular.z = 0.0
        elif 0.4 < sum(range_ahead[:30]) / 30 < 0.8:
            twist.linear.x = 0.10
            twist.angular.z = 0.15
        elif 0.4 < sum(range_ahead[60:]) / 30 < 0.8:
            twist.linear.x = 0.10
            twist.angular.z = -0.15
        elif 0.4 < sum(range_ahead[:15]) / 30 < 0.8:
            twist.linear.x = 0.10
            twist.angular.z = 0.15
        elif 0.4 < sum(range_ahead[:75]) / 30 < 0.8:
            twist.linear.x = 0.10
            twist.angular.z = -0.15
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)

    def follow(self):
        while not rospy.is_shutdown():

            self.laser_scan()

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)


def main():

    rospy.init_node('follower', anonymous=True)

    try:
        follow = Follower()
    except rospy.ROSInterruptException:
        print("Shutting down")


if __name__ == '__main__':
    main()
