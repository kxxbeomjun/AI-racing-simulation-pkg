#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from predict_dyn_obs_path.msg import Trajectory
import numpy as np

class TrajectoryPredictor:

    def __init__(self):
        rospy.init_node('trajectory_predict_RLS')
        self.rate = rospy.Rate(10)

        self.subscriber = rospy.Subscriber('/erp42_1/ground_truth_pose', Odometry, self.pose_callback)
        self.publisher = rospy.Publisher('/erp42_1/predicted_trajectory', Trajectory, queue_size=10)

        # Initialize RLS parameters
        self.alpha = 0.99
        self.n = 2
        self.p = 0.1*np.eye(self.n)
        self.theta = np.zeros((self.n, 1))

    def pose_callback(self, msg):
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

        # Update the RLS parameters
        x = position.reshape((self.n, 1))
        y = position[0]
        k = np.dot(self.p, x) / (self.alpha + np.dot(x.T, np.dot(self.p, x)))
        self.theta += k * (y - np.dot(x.T, self.theta))
        self.p = (self.p - np.dot(k, np.dot(x.T, self.p))) / self.alpha

        # Estimate the trajectory
        t = np.arange(0, 10, 0.1)  # Time vector
        X = np.column_stack((np.ones(t.size), t))  # Design matrix
        y_pred = np.dot(X, self.theta)  # Predicted trajectory

        traj_msg = Trajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.trajectory = y_pred.flatten().tolist()

        self.publisher.publish(traj_msg)

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
