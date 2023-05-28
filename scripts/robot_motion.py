#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import random
import numpy
import math

class RobotMotion:
    def __init__(self) -> None:
        rospy.init_node('robot_moion', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        self.forward = Twist()
        self.forward.linear.x = 0.5
        self.stop = Twist()
        self.stop.linear.x = 0.0
        self.right = Twist()
        self.right.angular.z = -0.5


    def move_circle(self):
        circle = Twist()
        circle.linear.x = 0.2
        circle.angular.z = 0.5
        while not rospy.is_shutdown():
            self.cmd_pub.publish(circle)
            rospy.sleep(0.1)
    
    def move_square(self):
        self.cmd_pub.publish(self.forward)
        rospy.sleep(2.0)
        self.cmd_pub.publish(self.stop)
        rospy.sleep(0.5)
        for i in range(4):
            self.cmd_pub.publish(self.forward)
            rospy.sleep(1.0)
            self.cmd_pub.publish(self.stop)
            rospy.sleep(0.5)
            self.cmd_pub.publish(self.right)
            rospy.sleep(2.0)
            self.cmd_pub.publish(self.stop)
            rospy.sleep(0.5)

        self.cmd_pub.publish(self.stop)

    
    def move_random(self):
        random_msg = Twist()
        # random_msg.linear.x = random.uniform(-0.5, 0.5)
        # random_msg.angular.z = random.uniform(-0.5, 0.5)
        while not rospy.is_shutdown():
            random_msg.linear.x = random.uniform(0.0, 0.5)
            random_msg.angular.z = random.uniform(-0.5, 0.5)
            print(random_msg.linear.x, random_msg.angular.z)
        
            self.cmd_pub.publish(random_msg)
            rospy.sleep(0.1)


    def shutdown(self):
        self.cmd_pub.publish(Twist())


if __name__ == '__main__':
    try:
        robot = RobotMotion()
        robot.move_random()

    except rospy.ROSInterruptException:
        robot.shutdown()