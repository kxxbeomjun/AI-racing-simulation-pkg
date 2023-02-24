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
from path_extraction.srv import StartRacing, StartRacingRequest, ResetAndRestore, ResetAndRestoreResponse
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState


class RefereeServer:
    def __init__(self):
        #init node 
        rospy.init_node('RefereeServer', anonymous=True)
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

    def reset_and_restore(self, request):
        try:

            ## Stop the cars 
            request = StartRacingRequest()
            request.start_sign = False
            for client in self.movebase_follower_clients:
                client(request)
            print("all the cars are stop")

            #Wait a second
            rospy.sleep(3)

            ## Move to the nearest point in the path 
            state_list = list()
            for i in range(self.vehicle_num):
                pose_idx = int(self.poses_idx[i])
                state_msg = ModelState()
                state_msg.model_name = 'erp42_' + str(i+1)
                state_msg.pose.position.x = self.global_path.poses[pose_idx].pose.position.x
                state_msg.pose.position.y = self.global_path.poses[pose_idx].pose.position.y
                state_msg.pose.position.z = 0
                adjacent_point_index = pose_idx - 1
                if adjacent_point_index > len(self.global_path.poses)-1:
                    adjacent_point_index = adjacent_point_index - len(self.global_path.poses)
                    
                goal_yaw = atan2(state_msg.pose.position.y-self.global_path.poses[adjacent_point_index].pose.position.y,
                            state_msg.pose.position.x-self.global_path.poses[adjacent_point_index].pose.position.x)
                quaternion = tf.transformations.quaternion_from_euler(0,0,goal_yaw)
                state_msg.pose.orientation.x = quaternion[0]
                state_msg.pose.orientation.y = quaternion[1]
                state_msg.pose.orientation.z = quaternion[2]
                state_msg.pose.orientation.w = quaternion[3]
                state_list.append(state_msg)
            rospy.wait_for_service('/gazebo/set_model_state')
            
            for i in range(self.vehicle_num):
                self.gazebo_clients(state_list[i])
            
            ## Start the cars 
            request = StartRacingRequest()
            request.start_sign = True
            for client in self.movebase_follower_clients:
                client(request)
            print("restart all the cars")
            result = True
        except Exception as e:
            print("Exection occured : ", e)
            result = False


        return ResetAndRestoreResponse(result)

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
        self.gazebo_clients = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        self.movebase_follower_client_1 = rospy.ServiceProxy('erp42_1/start_racing', StartRacing)
        self.movebase_follower_client_2 = rospy.ServiceProxy('erp42_2/start_racing', StartRacing)
        self.movebase_follower_clients = [self.movebase_follower_client_1, self.movebase_follower_client_2]
        self.reset_service = rospy.Service('reset_and_restore', ResetAndRestore, self.reset_and_restore)
    
    def callback(self, config):
        return config
if __name__ == '__main__':
    RefereeServer()

