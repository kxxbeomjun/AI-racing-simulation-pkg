#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from predict_dyn_obs_path.msg import Trajectory
import numpy as np
from filterpy.kalman import KalmanFilter

class TrajectoryPredictor:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('trajectory_predict_Kalman')
        self.rate = rospy.Rate(20)

        self.subscriber = rospy.Subscriber('/erp42_1/ground_truth_pose', Odometry, self.pose_callback)
        self.publisher = rospy.Publisher('/erp42_1/predicted_trajectory', Trajectory, queue_size=1)

        # Initialize Kalman filter parameters
        self.dt = 0.1
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.x = np.array([0., 0., 0., 0.])  # Initial state (x, y, vx, vy)
        self.kf.F = np.array([[1., 0., self.dt, 0.], [0., 1., 0., self.dt], [0., 0., 1., 0.], [0., 0., 0., 1.]])  # State transition matrix
        self.kf.H = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.]])  # Measurement matrix
        self.kf.P *= 1000  # Covariance matrix
        self.kf.R = np.array([[0.1, 0.], [0., 0.1]])  # Measurement noise covariance
        self.kf.Q = np.array([[self.dt**4/4, 0., self.dt**3/2, 0.], [0., self.dt**4/4, 0., self.dt**3/2], [self.dt**3/2, 0., self.dt**2, 0.], [0., self.dt**3/2, 0., self.dt**2]])  # Process noise covariance

    def pose_callback(self, msg):
        # Extract the position from the Odometry message
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

        # Update the Kalman filter with the new measurement
        self.kf.predict()
        self.kf.update(position)

        # Estimate the trajectory
        t = np.arange(1, 5, self.dt)  # Time vector
        X = np.column_stack((self.kf.x[0] + self.kf.x[2]*t, self.kf.x[1] + self.kf.x[3]*t))  # Predicted trajectory

        traj_msg = Trajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.trajectory = X.flatten().tolist()

        self.publisher.publish(traj_msg)
        print(traj_msg.trajectory[-2], traj_msg.trajectory[-1])

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

            self.rate.sleep()

if __name__ == '__main__':
	try:
		predictor = TrajectoryPredictor()
		predictor.run()
	except rospy.ROSInterruptException:
		pass
      