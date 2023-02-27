
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry 

import numpy as np
from numpy.linalg import inv

import math

# np.random.seed(0)

# Input parameters.

freq = 20 
mean_size = 5 
mean_count = 0
dt= 1.0/freq*mean_size
meas_noise_size=10
sys_noise_size=10

z_mean=np.zeros((2,mean_size))

# Initialization for system model.
# Matrix: A, H, Q, R, P_0
# Vector: x_0
A = np.array([[1,dt, 0 ,0],
              [0, 1, 0 ,0],
              [0, 0, 1,dt],
              [0, 0, 0, 1]])  #system model variable
H = np.array([[1, 0, 0, 0],
              [0, 0, 1, 0]])  #system model vriable
#Q = np.array([[1, 0, 0, 0],   
#              [0, 1, 0, 0],
#              [0, 0, 1, 0],
#              [0, 0, 0, 1]])  #system noise constant
Q = 10 * np.eye(4) 
R = np.array([[100]])    #sensor noise constant
# Initialization for estimation.
x_0 = np.array([-177, 0, -33, 0]) # [x, u, y, v] #first position erp42_1 : x:-177, y:-33
#P_0 = 1 * np.eye(4)     #if first step, let p large
P_0= np.array(    [[  23.55254745,   19.55088905,   23.55254745,   19.55088905],
                  [  19.55088905,  222.99915521,   19.55088905,   17.93666497],
                  [  23.55254745,   19.55088905,   23.55254745,   19.55088905],
                  [  19.55088905,   17.93666497,   19.55088905,  222.99915521]])

x_esti= x_0
P = P_0
z_0=np.array([[0, 0, 0],[0, 0, 0]])

def kalman_filter(z_meas, x_esti, P):
    #"""Kalman Filter Algorithm."""
    # (1) Prediction.
    x_pred = A.dot(x_esti)
    P_pred = A.dot(P.dot(A.T)) + Q

    # (2) Kalman Gain.
    K = P_pred.dot(H.T.dot(inv(H.dot(P_pred.dot(H.T)) + R)))

    # (3) Estimation.
    x_esti = x_pred + K.dot(z_meas - H.dot(x_pred))

    # (4) Error Covariance.
    P = P_pred - K.dot(H.dot(P_pred))

    return x_esti, P

def callback(data):
      global x_esti, P, sys_noise_size, meas_noise_size
      global mean_size, mean_count, z_mean

      real_x=data.pose.pose.position.x
      real_y=data.pose.pose.position.y
      #noise
      w = np.random.normal(0, np.sqrt(sys_noise_size))      # w: system noise.
      v_1 = np.random.normal(0, np.sqrt(meas_noise_size))      # v: measurement noise.
      v_2 = np.random.normal(0, np.sqrt(meas_noise_size))      # v: measurement noise.

      #truth value + noise
      x_pose=data.pose.pose.position.x+v_1
      y_pose=data.pose.pose.position.y+v_1
      
      mean_count=1+mean_count
      if(mean_count<=mean_size):
            z_mean=np.append(z_mean,[[x_pose],[y_pose]],axis=1)
            z_mean=np.delete(z_mean,0,axis=1)
            #print(z_mean)

      if(mean_count>=mean_size):          
            mean_count=0
            #kalman filter
            #z_meas=np.array([x_pose,y_pose]).T
            z_meas=np.mean(z_mean,axis=1).T
            x_esti, P = kalman_filter(z_meas, x_esti, P)
            esti_vel=math.sqrt(x_esti[1]*x_esti[1]+x_esti[3]*x_esti[3])
            print("x:",x_esti[0],"/y:",x_esti[2],"/u:",x_esti[1],"/v:",x_esti[3],"/real_x:",real_x,"/real_y:",real_y,"/noise_x:",x_pose,"/noise_y:",y_pose)

def listener():
      rospy.init_node('listener',anonymous=True)
      x_esti, P = x_0, P_0
	
      rospy.Subscriber("/erp42_1/ground_truth_pose", Odometry , callback)

      rospy.spin()

if __name__=='__main__':
      listener()
```