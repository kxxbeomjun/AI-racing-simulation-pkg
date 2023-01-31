#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

from codecs import latin_1_decode
from locale import currency
from multiprocessing import current_process
import sys,os
import rospy
import rospkg
import numpy as np
# from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_srvs.srv import Empty ###############################

import tf
import tf2_ros
from math import cos,sin,sqrt,pow,atan2,pi
from tf.transformations import quaternion_from_euler
from lib.utils import pathReader, findLocalPath
import dynamic_reconfigure.client

class RankServer:
    def __init__(self):
        #init node 
        rospy.init_node('RankServer', anonymous=True)
        self.configure()

        # Path data reader
        path_reader = pathReader('path_extraction') ## 경로 파일의 패키지 위치
        self.global_path = np.array(path_reader.read_txt(self.path_file_name, self.path_frame)) ## 출력할 경로의 이름
        
        # ROS publsiher & subscriber 
        
        # time var

        self.rate = rospy.Rate(self.frequency)
        self.config = dict()

        while not rospy.is_shutdown():
            self.update_status()
            self.apply_speed_limit()
            self.rate.sleep()


    def update_status(self):

        poses = np.zeros(self.vehicle_num)
        for i in range(self.vehicle_num):
            x = rospy.get_param("/erp42_"+str(i+1)+"/pose_x")
            y = rospy.get_param("/erp42_"+str(i+1)+"/pose_y")
            map_idx = self.get_track_index(x,y)

            if not self.initial_check:
                if map_idx >= np.ceil(self.global_path.shape[0]/2):
                    self.laps[i] = -1
                    self.initial_check = True 
            
            self.poses_idx[i] = map_idx

            if np.abs(self.poses_idx[i]-self.prev_poses_idx[i]) >= np.ceil(self.global_path.shape[0]/2):
                self.laps[i] += 1 
            
            poses[i] = self.laps[i] * self.global_path.shape[0] + self.poses_idx[i]


        self.ranks = poses.argsort()
        self.prev_poses_idx = self.poses_idx

    def get_track_index(self, vehicle_idx, x, y):
        min_dis=float('inf')
        min_index = 0
        #First time : Check all of the path 
        for i in range(self.global_path.shape[0]):
            dx = x - self.global_path.poses[i].pose.position.x
            dy = y - self.global_path.poses[i].pose.position.y
            dis = sqrt(dx*dx + dy*dy)
            if dis < min_dis :
                min_dis = dis
                min_index = i
        
        return min_index

    def apply_speed_limit(self):

        for i, rank in enumerate(self.ranks):
            if rank == np.max(self.ranks):
                rospy.set_param('/erp42_'+str(i+1)+/'/max_allowed_velocity', 4.0)
            else:
                rospy.set_param('/erp42_'+str(i+1)+/'/max_allowed_velocity', 5.0)


    def configure(self):
        self.path_file_name = rospy.get_param("~path_file_name")
        self.frequency = rospy.get_param("~frequency")
        self.vehicle_num = int(rospy.get_param("~vehicle_num"))

        self.ranks = np.zeros(self.vehicle_num)
        self.laps = np.zeros(self.vehicle_num)
        self.poses_idx = np.zeros(self.vehicle_num)
        self.prev_poses_idx = np.zeros(self.vehicle_num)

if __name__ == '__main__':
    RankServer()

