#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from math import atan2, sin, cos, tan
from numpy import array, sqrt
import numpy as np

class EKFNode:
    def __init__(self):
        

        self.ekf = ExtendedKalmanFilter(dim_x=6, dim_z=4)
        dt = 0.01  # time step

        self.imu_yaw = 0

        # Set the initial state
        x_init = array([[0], [0], [0], [0], [0], [0]])  # Include vx, vy, and vtheta
        self.ekf.x = x_init

        # Set the initial state covariance
        P = 500
        if np.isscalar(P):
            self.ekf.P *= P
        else:
            self.ekf.P[:] = P

        # Set the process noise covariance matrix
        Q = 0.01
        if np.isscalar(Q):
            self.ekf.Q *= Q
        else:
            self.ekf.Q[:] = Q

        # Set the measurement noise covariance matrix
        R = np.diag([500.0, 500.0, 500.0, 0.1])  # Assuming independent noise for x, y, theta, and imu_yaw measurements
        self.ekf.R *= R

        self.F = self.F_jacobian(self.ekf.x, dt)  # Compute the initial Jacobian matrix
        self.ekf.F = self.F  # Assign the Jacobian matrix to ekf.F

        self.odom_filtered_pub = rospy.Publisher('/odom_filtered', Odometry, queue_size=10)

        rospy.Subscriber('/odom_noisy', Odometry, self.odom_callback)
        rospy.Subscriber('/sk/imu', Imu, self.imu_callback)

    def motion_model(self, x, dt):
        theta = x[2, 0]
        vx = x[3, 0]  # Velocity in x direction
        vy = x[4, 0]  # Velocity in y direction
        vtheta = x[5, 0]  # Angular velocity

        x[0, 0] += dt * (vx * cos(theta) - vy * sin(theta))
        x[1, 0] += dt * (vx * sin(theta) + vy * cos(theta))
        x[2, 0] += dt * vtheta
        return x

    def F_jacobian(self, x, dt):
        theta = x[2, 0]
        R_wheels = 0.6
        d = sqrt(x[0, 0] ** 2 + x[1, 0] ** 2)  # Distance traveled
        alpha = atan2(x[1, 0], x[0, 0])  # Current heading
        beta = (d / R_wheels) * tan(alpha)
        return np.array([[1, 0, -R_wheels * cos(theta) + R_wheels * cos(theta + beta), dt, 0, 0],
                         [0, 1, -R_wheels * sin(theta) + R_wheels * sin(theta + beta), 0, dt, 0],
                         [0, 0, 1, 0, 0, dt],
                         [0, 0, 0, 1, 0, 0],
                         [0, 0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 0, 1]])
    
        # return np.array([[1, 0, 0, dt, 0, 0],
        #                  [0, 1, 0, 0, dt, 0],
        #                  [0, 0, 1, 0, 0, dt],
        #                  [0, 0, 0, 1, 0, 0],
        #                  [0, 0, 0, 0, 1, 0],
        #                  [0, 0, 0, 0, 0, 1]])
    def Hx(self, x):
        return np.array([[x[0, 0]], [x[1, 0]], [x[2, 0]], [x[5, 0]]])  # Return x, y, theta, and imu_yaw

    def H_jacobian(self, x):
        return array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 0, 0, 1]])

    def residual(self, a, b):
        y = a - b
        y[2] = atan2(sin(y[2]), cos(y[2]))
        return y

    def odom_callback(self, msg):
        # Extract odometry measurements from the message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = atan2(2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                      1 - 2 * (msg.pose.pose.orientation.y ** 2 + msg.pose.pose.orientation.z ** 2))
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vtheta = msg.twist.twist.angular.z

        # Set up the measurement vector for odometry
        z = array([[x], [y], [theta], [self.imu_yaw]])  # Set imu_yaw as 0 initially

        # Perform the prediction step
        self.ekf.predict()

        # Perform the update step with odometry measurement
        self.ekf.update(z, HJacobian=self.H_jacobian, Hx=self.Hx)

        self.publish_filtered_odom()


    def imu_callback(self, msg):
        # Extract IMU measurements from the message
        self.imu_yaw = atan2(2 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y),
                        1 - 2 * (msg.orientation.y ** 2 + msg.orientation.z ** 2))


    def publish_filtered_odom(self):
        # Publish the filtered odometry data
        filtered_odom = Odometry()
        filtered_odom.header.stamp = rospy.Time.now()
        filtered_odom.header.frame_id = "odom_filtered"
        filtered_odom.pose.pose.position.x = self.ekf.x[0, 0]
        filtered_odom.pose.pose.position.y = self.ekf.x[1, 0]
        filtered_odom.pose.pose.orientation.z = sin(self.ekf.x[2, 0] / 2)
        filtered_odom.pose.pose.orientation.w = cos(self.ekf.x[2, 0] / 2)
        filtered_odom.twist.twist.linear.x = self.ekf.x[3, 0]
        filtered_odom.twist.twist.linear.y = self.ekf.x[4, 0]
        filtered_odom.twist.twist.angular.z = self.ekf.x[5, 0]
        self.odom_filtered_pub.publish(filtered_odom)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('filter')
    try:
        node = EKFNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
