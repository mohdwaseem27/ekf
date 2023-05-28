# Extended Kalman Filter (EKF) for state estimation

This package contains an implementation of an Extended Kalman Filter (EKF) for state estimation using odometry and IMU measurements. The EKF estimates the position and velocity of a mobile robot in a 2D space.

## Dependencies

The following dependencies are required to run the code:

- ROS1 (Robot Operating System)
- Python (version 3.x)
- numpy
- filterpy

## Installation

1. Install ROS by following the official installation instructions: [ROS Installation](http://wiki.ros.org/ROS/Installation)

2. Install the required Python packages using pip:

   ```
   pip install numpy filterpy
   ```


3. Clone this package to your ROS workspace:

   ```
   cd <path_to_your_ros_workspace>/src
   git clone https://github.com/mohdwaseem27/ekf.git
   ```

4. Build and source the ROS workspace:
   ```
   cd <path_to_your_ros_workspace>
   catkin_make
   source devel/setup.bash
   ```

## Usage
1. Launch the robot:
   ```
   roslaunch ekf ekf.launch
   ```

2. Add some noise to the odom:
   ```
   rosrun ekf odom_noise.py
   ```
3. Run the EKF node:
   ```
   rosrun ekf ekf.py
   ```
4. Subscribe to the `/odom_filtered` topic to receive the filtered odometry data.

5. Publish the odometry and IMU messages to the respective topics (`/odom_noisy` and `/sk/imu`) for the EKF to process.


## Visualizing Ground Truth and Filtered Odometry
![Graph of Ground Truth and Filtered Odometry](videos/ekf.gif)

![Graph of Ground Truth, Filtered Odometry, and Noisy Odometry](videos/ekf_noise.gif)

## Subscribers and Publishers
- Subscribers:
   - `/odom_noisy` (nav_msgs/Odometry): Odometry data with noise
   - `/sk/imu` (sensor_msgs/Imu): IMU measurements
- Publishers:
   - `/odom_filtered` (nav_msgs/Odometry): Filtered odometry data.
## Configuration

- The initial state of the EKF and the initial state covariance can be adjusted in the `__init__` function of the EKFNode class.
- The process noise covariance matrix (Q) and the measurement noise covariance matrix (R) can be modified in the `__init__` function as well.

## Customization

- If needed, you can modify the motion model (motion_model function), the measurement model (Hx function), and their respective Jacobian matrices (F_jacobian and H_jacobian functions) to suit your specific application.
- Additional sensor data or measurements can be incorporated into the EKF by adding new subscriber callbacks and modifying the measurement vector (z) and covariance matrices accordingly.
