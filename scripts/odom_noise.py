#!/usr/bin/env python3
import rospy
import math
import sys
import numpy as np
from nav_msgs.msg import Odometry

class Noise:

    def __init__(self):
        self.source_topic = '/odom'
        self.published_topic = '/odom_noisy'
        self.position_noise = float(0.5) 
        self.velocity_noise = float(0.5) 
        self.odom = Odometry()

        rate = rospy.Rate(200)
        rospy.Subscriber(self.source_topic, Odometry, self.source_callback)
        self.pub = rospy.Publisher(self.published_topic, Odometry, queue_size=10)

        while not rospy.is_shutdown():
            self.pub.publish(self.odom)
            rate.sleep()

    def source_callback(self, odom):
    	self.odom.header = odom.header
    	self.odom.child_frame_id = odom.child_frame_id
    	self.odom.pose.pose.position.x = odom.pose.pose.position.x + np.random.normal(0, self.position_noise)
    	self.odom.pose.pose.position.y = odom.pose.pose.position.y + np.random.normal(0, self.position_noise)
    	self.odom.pose.pose.orientation.x = odom.pose.pose.orientation.x #+ np.random.normal(0, self.position_noise)
    	self.odom.pose.pose.orientation.y = odom.pose.pose.orientation.y #+ np.random.normal(0, self.position_noise)
    	self.odom.pose.pose.orientation.z = odom.pose.pose.orientation.z #+ np.random.normal(0, self.position_noise)
    	self.odom.pose.pose.orientation.w = odom.pose.pose.orientation.w #+ np.random.normal(0, self.position_noise)
    	self.odom.twist.twist.linear.x = odom.twist.twist.linear.x #+ np.random.normal(0, self.velocity_noise) 
    	self.odom.twist.twist.angular.z = odom.twist.twist.angular.z# + np.random.normal(0, self.velocity_noise) 
    	self.odom.pose.covariance = odom.pose.covariance



if __name__ == '__main__':
    rospy.init_node("Noise")
    try:
        node = Noise()
    except rospy.ROSInterruptException:
        pass



'''#!/usr/bin/env python

''
Modified version of the original from: Team Leonard, University of Birmingham Intelligent Robotics 2018
''

import rospy
import math
import random
from random import gauss
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_srvs.srv import Empty, EmptyResponse
# sl = standard deviation of the linear velocity Gaussian noise
# sa = standard deviation of the angular velocity Gaussian noise
sl, sa = 0.1, 0.5


def rotateQuaternion(q_orig, yaw):
    """
    Converts a basic rotation about the z-axis (in radians) into the
    Quaternion notation required by ROS transform and pose messages.

    :Args:
       | q_orig (geometry_msgs.msg.Quaternion): to be rotated
       | yaw (double): rotate by this amount in radians
    :Return:
       | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
     """
    # Create a temporary Quaternion to represent the change in heading
    q_headingChange = Quaternion()

    p = 0
    y = yaw / 2.0
    r = 0

    sinp = math.sin(p)
    siny = math.sin(y)
    sinr = math.sin(r)
    cosp = math.cos(p)
    cosy = math.cos(y)
    cosr = math.cos(r)

    q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
    q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
    q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
    q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

    # Multiply new (heading-only) quaternion by the existing (pitch and bank)
    # quaternion. Order is important! Original orientation is the second
    # argument rotation which will be applied to the quaternion is the first
    # argument.
    return multiply_quaternions(q_headingChange, q_orig)


def multiply_quaternions( qa, qb ):
    """
    Multiplies two quaternions to give the rotation of qb by qa.

    :Args:
       | qa (geometry_msgs.msg.Quaternion): rotation amount to apply to qb
       | qb (geometry_msgs.msg.Quaternion): to rotate by qa
    :Return:
       | (geometry_msgs.msg.Quaternion): qb rotated by qa.
    """
    combined = Quaternion()

    combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
    combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
    combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
    combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
    return combined


def getHeading(q):
    """
    Get the robot heading in radians from a Quaternion representation.

    :Args:
        | q (geometry_msgs.msg.Quaternion): a orientation about the z-axis
    :Return:
        | (double): Equivalent orientation about the z-axis in radians
    """
    yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw


def simple_gaussian(odom):
    """
    Applies simple gaussian noise to current position and odometry readings.
    """
    sp, sr = 0.01, 0.008
    pos = odom.pose.pose.position
    odom.pose.pose.position = Point(gauss(pos.x, sp), gauss(pos.y, sp), gauss(pos.z, sp))
    rot = odom.pose.pose.orientation
    odom.pose.pose.orientation = Quaternion(gauss(rot.x, sr), gauss(rot.y, sr), gauss(rot.z, sr), gauss(rot.w, sr))
    return odom


def closed_loop_problem(odom):
    ''
        using the linear and angular velocities extracted from each odometry
        message: add noise to these velocities and add them to the current
        fictional position to keep track independently of the positions reported
        by the odometry.
    ''
    global cl_odom

    # If cl_odom is not defined, then it must be the first callback
    if 'cl_odom' not in globals():
        cl_odom = odom
    else:
        # Get velocities
        lv = odom.twist.twist.linear
        av = odom.twist.twist.angular
        dt = (odom.header.stamp - cl_odom.header.stamp).nsecs * 1e-9

        # Add noise to velocities
        lv = Vector3(gauss(lv.x, sl), gauss(lv.y, sl), lv.z)
        av = Vector3(av.x, av.y, gauss(av.z, av.z * sa))

        # Apply velocities to orientation of last location
        cl_ori = cl_odom.pose.pose.orientation
        odom.pose.pose.orientation = rotateQuaternion(cl_ori, av.z * dt)
        odom.twist.twist.angular = av
        yaw = getHeading(odom.pose.pose.orientation) % (2 * math.pi)

        # Apply velocities to position of last location
        cl_pos = cl_odom.pose.pose.position
        fwd, drift = lv.x * dt, lv.y * dt
        c = math.cos(yaw)
        s = math.sin(yaw)
        odom.pose.pose.position.x = cl_pos.x + c * fwd + s * drift
        odom.pose.pose.position.y = cl_pos.y + s * fwd + c * drift
        odom.twist.twist.linear = lv

        # Set cl_odom to odom
        cl_odom = odom


def odometry_callback(odom):
    global pub
    rospy.logdebug("Got perfect odom: {} {}".format(odom.pose.pose.position.x, odom.pose.pose.position.y)) 
    # Add noise to the odometry
    closed_loop_problem(odom)
    # Republish
    pub.publish(odom)
    rospy.logdebug("Pub noisy odom: {} {}".format(odom.pose.pose.position.x, odom.pose.pose.position.y)) 

def shutdown_callback(_):
    global shutdown_flag
    response = EmptyResponse()
    # set shutdown flag
    shutdown_flag = True
    return response

def clean_shutdown():
    rospy.loginfo("Shutting down noisy odometry node...")

if __name__ == '__main__':
    global pub, shutdown_flag
    shutdown_flag = False
    rospy.init_node('noisy_odom')

    pub = rospy.Publisher('odom_noisy', Odometry, queue_size=1)
    rospy.Subscriber('odom', Odometry, odometry_callback)
    shutdown_service = rospy.Service('/noisy_odom/shutdown', Empty, shutdown_callback)
    rospy.loginfo("Started noisy odometry publisher node")
    # cleanup on shutdown
    rospy.on_shutdown(clean_shutdown)
    # equivalent to spin()
    while not rospy.core.is_shutdown() and not shutdown_flag:
      rospy.rostime.wallsleep(0.5)
    rospy.Timer(rospy.Duration(1), rospy.signal_shutdown('Shutting down'), oneshot=True)'''
