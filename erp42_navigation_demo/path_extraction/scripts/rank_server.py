#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import sys,os
import rospy
import rospkg
import numpy as np
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
        self.global_path = path_reader.read_txt(self.path_file_name, self.path_frame) ## 출력할 경로의 이름
        self.config = dict()

        while not rospy.is_shutdown():
            self.update_status()
            self.apply_speed_limit()
            self.rate.sleep()


    def update_status(self):
        
            poses = np.zeros(self.vehicle_num)
            for i in range(self.vehicle_num):
                try : 
                    x = rospy.get_param("/erp42_"+str(i+1)+"/pose_x")
                    y = rospy.get_param("/erp42_"+str(i+1)+"/pose_y")
                except:
                    print("Can't get the pose of the vehicle")
                    return
                map_idx = self.get_track_index(i, x, y)
                self.poses_idx[i] = map_idx
                poses[i] = self.laps[i] * len(self.global_path.poses) + self.poses_idx[i]
            

            self.ranks = poses.argsort()
            self.prev_poses_idx = self.poses_idx

    def get_track_index(self, vehicle_idx, x, y):
        min_dis=float('inf')
    
        #First time : Check all of the path 
        if not self.initial_check[vehicle_idx]:
            min_index = 0
            for i in range(len(self.global_path.poses)):
                dx = x - self.global_path.poses[i].pose.position.x
                dy = y - self.global_path.poses[i].pose.position.y
                dis = sqrt(dx*dx + dy*dy)
                if dis < min_dis :
                    min_dis = dis
                    min_index = i
    
            if min_index >= np.ceil(len(self.global_path.poses)/2):
                self.laps[vehicle_idx] = -1
            self.initial_check[vehicle_idx] = True 

        #Not the first time : Check from the previous index 
        else:
            min_index = int(self.prev_poses_idx[vehicle_idx])
            for i in range(min_index, min_index+self.look_ahead_index):
                if i >= len(self.global_path.poses):
                    i -= len(self.global_path.poses)
                dx = x - self.global_path.poses[i].pose.position.x
                dy = y - self.global_path.poses[i].pose.position.y
                dis = sqrt(dx*dx + dy*dy)
                if dis < min_dis :
                    min_dis = dis
                    min_index = i

            if np.abs(min_index-self.prev_poses_idx[vehicle_idx]) >= np.ceil(len(self.global_path.poses)/2):
                    self.laps[vehicle_idx] += 1 

        return min_index

    def apply_speed_limit(self):

        for i, rank in enumerate(self.ranks):
            if rank == np.max(self.ranks):
                self.clients[i].update_configuration({"max_allowed_velocity":4.0})
                rospy.set_param('/erp42_'+str(i+1)+'/move_base/RegulatedPurePursuitController/max_allowed_velocity', 4.0)
            else:
                self.clients[i].update_configuration({"max_allowed_velocity":5.0})
                rospy.set_param('/erp42_'+str(i+1)+'/move_base/RegulatedPurePursuitController/max_allowed_velocity', 5.0)


    def configure(self):
        #Get ROS Parameters
        self.path_file_name = rospy.get_param("~path_file_name")
        self.path_frame = rospy.get_param("~path_frame")
        self.vehicle_num = int(rospy.get_param("~vehicle_num"))

        #Initialize variables
        self.rate = rospy.Rate(1)
        self.ranks = np.zeros(self.vehicle_num)
        self.laps = np.zeros(self.vehicle_num)
        self.poses_idx = np.zeros(self.vehicle_num)
        self.prev_poses_idx = np.zeros(self.vehicle_num,dtype=int)
        self.look_ahead_index = 30
        self.initial_check = np.full((self.vehicle_num,), False)
        self.client_1 = dynamic_reconfigure.client.Client("erp42_1/move_base/RegulatedPurePursuitController", timeout=30, config_callback=self.callback)
        self.client_2 = dynamic_reconfigure.client.Client("erp42_2/move_base/RegulatedPurePursuitController", timeout=30, config_callback=self.callback)
        self.clients = [self.client_1, self.client_2]
    
    def callback(self, config):
        return config
if __name__ == '__main__':
    RankServer()

