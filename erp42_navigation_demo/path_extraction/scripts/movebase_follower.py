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
from path_extraction.srv import StartRacing, StartRacingResponse

class MoveBaseFollower:
    def __init__(self):
        #init node 
        rospy.init_node('movebase_follower', anonymous=True)
        self.configure()
        self.get_current_idx_ = False
        self.current_waypoint = 0
        self.zone_idx = 'init'
        self.zone_idx_last = -1
        self.isCustom = True
        self.desired_linear_vel_ = 3.0

        # Path data reader
        path_reader = pathReader('path_extraction') ## 경로 파일의 패키지 위치
        self.global_path = path_reader.read_txt(self.path_file_name, self.path_frame) ## 출력할 경로의 이름
        
        # ROS publsiher & subscriber 
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1) ## Vehicle Control
        # time var
        self.rate = rospy.Rate(self.frequency)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.status_msg = PoseStamped()
        
        self.client = dynamic_reconfigure.client.Client("move_base/RegulatedPurePursuitController", timeout=30, config_callback=self.configCallback)
        self.config = dict()
        print("Creating service")
        self.service = rospy.Service('start_racing', StartRacing, self.start_racing)
        print("Creating service done")
        while not rospy.is_shutdown():
            self.update_status()
            self.send_goal()
            self.rate.sleep()

    def configCallback(self,config):
        return

    def reconfigure(self, client):
        self.config["desired_linear_vel"] = self.desired_linear_vel_
        client.update_configuration(self.config)

    def update_status(self):
        tf_recieved = False
        #print("self.path_frmame:%s, self.robot_frame:%s"%(self.path_frame, self.robot_frame))
        while not tf_recieved:
            try:
                trans = self.tfBuffer.lookup_transform(self.path_frame, self.robot_frame, rospy.Time())
                #print("tf received")
                tf_recieved = True
                
                self.status_msg.pose.position.x = trans.transform.translation.x
                self.status_msg.pose.position.y = trans.transform.translation.y
                self.status_msg.pose.position.z = trans.transform.translation.z
                self.status_msg.pose.orientation.x = trans.transform.rotation.x
                self.status_msg.pose.orientation.y = trans.transform.rotation.y
                self.status_msg.pose.orientation.z = trans.transform.rotation.z
                self.status_msg.pose.orientation.w = trans.transform.rotation.w
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def configure(self):
        self.path_file_name = rospy.get_param("~path_file_name")
        self.path_frame = rospy.get_param("~path_frame")
        self.robot_frame = rospy.get_param("~robot_frame")
        self.frequency = rospy.get_param("~frequency")
        self.lai = rospy.get_param("~look_ahead_index")
        self.start_sign = rospy.get_param("~start_sign", True)
        # rospy.set_param('/move_base/PurePursuitPlannerROS/look_ahead_distance', 15.0)
        # /move_base/PurePursuitPlannerROS/look_ahead_distance

    def send_goal(self):
        #print("self.current_waypoint", self.current_waypoint)
        ## Local path중 look ahead distance 앞에 있는 점을 move base goal로 만듬 
        vehicle_pose = self.status_msg.pose.position

        current_x = vehicle_pose.x
        current_y = vehicle_pose.y
        min_dis=float('inf')
        min_index = 0
        #First time : Check all of the path 
        if not self.get_current_idx_:
            for i in range(len(self.global_path.poses)):
                dx = current_x - self.global_path.poses[i].pose.position.x
                dy = current_y - self.global_path.poses[i].pose.position.y
                dis = sqrt(dx*dx + dy*dy)
                if dis < min_dis :
                    min_dis = dis
                    min_index = i
            self.get_current_idx_ = True
        #Not a First time : Check last index ~ last goal index 
        else: 
            for i in range(self.current_waypoint,self.current_waypoint + self.lai):
                if i > len(self.global_path.poses)-1:
                    i = i - len(self.global_path.poses)
                dx = current_x - self.global_path.poses[i].pose.position.x
                dy = current_y - self.global_path.poses[i].pose.position.y
                dis = sqrt(dx*dx + dy*dy)
                if dis < min_dis :
                    min_dis = dis
                    min_index = i

        self.current_waypoint = min_index

        if self.start_sign:
            goal_index = self.current_waypoint + self.lai
            adjacent_point_index = goal_index - 1

            if goal_index > len(self.global_path.poses)-1:
                goal_index = goal_index - len(self.global_path.poses)
            
            if adjacent_point_index > len(self.global_path.poses)-1:
                adjacent_point_index = adjacent_point_index - len(self.global_path.poses)
            
            goal_point = self.global_path.poses[goal_index]
            goal_yaw = atan2(goal_point.pose.position.y-self.global_path.poses[adjacent_point_index].pose.position.y,
                            goal_point.pose.position.x-self.global_path.poses[adjacent_point_index].pose.position.x)

            self.goal = PoseStamped()
            self.goal.header.frame_id = "map"
            self.goal.header.stamp = rospy.Time.now()
            self.goal.pose.position.x = goal_point.pose.position.x
            self.goal.pose.position.y = goal_point.pose.position.y
            quaternion = tf.transformations.quaternion_from_euler(0,0,goal_yaw)
            self.goal.pose.orientation.x = quaternion[0]
            self.goal.pose.orientation.y = quaternion[1]
            self.goal.pose.orientation.z = quaternion[2]
            self.goal.pose.orientation.w = quaternion[3]

            
            self.goal_pub.publish(self.goal)

    def stop_command(self):
        stop_goal = PoseStamped()
        stop_goal.header.frame_id = "map"
        stop_goal.header.stamp = rospy.Time.now()
        stop_goal.pose.position.x = self.status_msg.pose.position.x
        stop_goal.pose.position.y = self.status_msg.pose.position.y
        stop_goal.pose.orientation.x = self.status_msg.pose.orientation.x
        stop_goal.pose.orientation.y = self.status_msg.pose.orientation.y
        stop_goal.pose.orientation.z = self.status_msg.pose.orientation.z
        stop_goal.pose.orientation.w = self.status_msg.pose.orientation.w
        self.goal_pub.publish(stop_goal)


    def start_racing(self, request):
        try:
            if request.start_sign:
                self.get_current_idx_ = False
                self.start_sign = True 
                print("Race Start!")
            else:
                self.update_status()
                self.stop_command()
                self.start_sign = False 
                print("Race Stop!")
            result = True
        except Exception as e:
            print("an error occurred: ", e)
            result = False
        return StartRacingResponse(result)



       
    

        
if __name__ == '__main__':
    MoveBaseFollower()

