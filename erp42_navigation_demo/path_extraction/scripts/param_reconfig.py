#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy
import rospkg
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_srvs.srv import Empty
from lib.utils import pathReader, findLocalPath
import dynamic_reconfigure.client
from math import cos,sin,sqrt,pow,atan2,pi,acos
import tf.transformations as tf
from numpy.linalg import inv

class ParamReconfig:

    def __init__(self):
        #init node 
        rospy.init_node('ParamReconfig', anonymous=True)
        self.configure()
        self.pose_subscriber()
        
        path_reader = pathReader('path_extraction')
        self.global_path = path_reader.read_txt(self.path_file_name, self.path_frame)
        self.config = dict()

        while not rospy.is_shutdown():
            self.update_status()
            self.apply_reconfigure()
            self.rate.sleep()


    def pose_subscriber(self):
        rospy.Subscriber("/erp42_1/ground_truth_pose", Odometry, self.callback1)
        rospy.Subscriber("/erp42_2/ground_truth_pose", Odometry, self.callback2)
        #rospy.Subscriber("/erp42_3/ground_truth_pose", Odometry, self.callback3)


    def callback1(self, data):
        if data is not None:
            self.poses[0] = data

    def callback2(self, data):
        if data is not None:
            self.poses[1] = data

    def callback3(self, data):
        if data is not None:
            self.poses[2] = data


    def update_status(self):
        
        for i in range(self.vehicle_num):
            try : 
                x = self.poses[i].pose.pose.position.x
                y = self.poses[i].pose.pose.position.y
                if self.initial_check_pose == False:
                    self.x_0[i][0] = x
                    self.x_0[i][2] = y
                    self.x_esti[i] = self.x_0[i]
                    self.initial_check_pose = True
            except:
                print("Can't get the pose of the vehicle")
                return
            map_idx = self.get_track_index(i, x, y)
            self.poses_idx[i] = map_idx
            if i == self.my_index - 1:
                self.my_pose[0] = self.poses[i].pose.pose.position.x
                self.my_pose[1] = self.poses[i].pose.pose.position.y
                quat = self.poses[i].pose.pose.orientation
                orientation = [quat.x, quat.y, quat.z, quat.w]
                euler = tf.euler_from_quaternion(orientation)
                self.my_pose[2] = euler[2]

        self.prev_poses_idx = self.poses_idx

        self.calc_right_or_left()
        self.calc_distance()
        self.calc_front_or_behind()
        self.calc_path_vector()
        self.calc_estimated_vel()
        self.calc_distance3()
        self.overtake_logic()
        self.calc_angle()
        


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

            self.initial_check[vehicle_idx] = True 

        #Not the first time : Check from the previous index 
        else:
            min_index = int(self.prev_poses_idx[vehicle_idx])
            for i in range(min_index, min_index+100):
                if i >= len(self.global_path.poses):
                    i -= len(self.global_path.poses)
                dx = x - self.global_path.poses[i].pose.position.x
                dy = y - self.global_path.poses[i].pose.position.y
                dis = sqrt(dx*dx + dy*dy)
                if dis < min_dis :
                    min_dis = dis
                    min_index = i

        return min_index


    def calc_distance(self):
        for i in range(self.vehicle_num):
            if i != self.my_index - 1:
                dx = self.poses[i].pose.pose.position.x - self.my_pose[0]
                dy = self.poses[i].pose.pose.position.y - self.my_pose[1]
                self.distance[i] = sqrt(dx*dx + dy*dy)
                if self.distance[i] < self.overtake_dist:
                    self.side_enabled[i] = True
                    self.front_enabled[i] = True
                    self.in_distance_counter[i] = self.in_distance_counter[i] + 0.1
                else:
                    self.side_enabled[i] = False
                    self.front_enabled[i] = False
                    self.in_distance_counter[i] = 0

    #if the front of two vehicles are near and we want to overtake
    def calc_angle(self):
        if self.car_angle[1]>0 and self.car_angle[2]>0:
            if self.distance[1]>self.distance[2]:
                self.side_enabled[1]=True
            else:
                self.side_enabled[2]=True

        if self.car_angle[1]<0 and self.car_angle[2]<0:
            if self.distance[1]>self.distance[2]:
                self.side_enabled[1]=True
            else:
                self.side_enabled[2]=True

    #if the vehicles distance are too close      
    def calc_distance3(self):
        for i in range(self.vehicle_num):   
            if i != self.my_index - 1:
                dx = self.poses[i].pose.pose.position.x - self.my_pose[0]
                dy = self.poses[i].pose.pose.position.y - self.my_pose[1]
                self.distance[i] = sqrt(dx*dx + dy*dy)
                if self.distance[i] < 2:
                    self.side_enabled[i] = False




    def calc_right_or_left(self):

        for i in range(self.vehicle_num):
            if i != self.my_index - 1:
                vec_1 = np.array([self.poses[i].pose.pose.position.x -self.global_path.poses[int(self.poses_idx[i]-5)].pose.position.x, 
                                self.poses[i].pose.pose.position.y - self.global_path.poses[int(self.poses_idx[i]-5)].pose.position.y])
                vec_2 = np.array([self.global_path.poses[int(self.poses_idx[i])].pose.position.x - self.global_path.poses[int(self.poses_idx[i]-5)].pose.position.x, 
                                self.global_path.poses[int(self.poses_idx[i])].pose.position.y - self.global_path.poses[int(self.poses_idx[i]-5)].pose.position.y])
                cross_result = np.cross(vec_1, vec_2)
                if cross_result > 0:
                    self.right_or_left[i] = True
                else:
                    self.right_or_left[i] = False


    def calc_front_or_behind(self):
        for i in range(self.vehicle_num):
            if i != self.my_index - 1:
                vec_1 = np.array([self.poses[i].pose.pose.position.x - self.my_pose[0], self.poses[i].pose.pose.position.y - self.my_pose[1]])
                vec_2 = np.array([cos(self.my_pose[2]), sin(self.my_pose[2])])
                dot_result = np.dot(vec_1, vec_2)
                if dot_result > 0:
                    self.front_or_behind[i] = True
                else:
                    self.front_or_behind[i] = False


    def calc_path_vector(self):
        for i in range(self.vehicle_num):
            if i != self.my_index - 1:
                dx = self.global_path.poses[int(self.poses_idx[i]+5)].pose.position.x - self.global_path.poses[int(self.poses_idx[i]-5)].pose.position.x
                dy = self.global_path.poses[int(self.poses_idx[i]+5)].pose.position.y - self.global_path.poses[int(self.poses_idx[i]-5)].pose.position.y
                self.path_vector[i] = float(atan2(dy, dx))


    def kalman_filter(self, z_meas, x_esti, P):
        x_pred = self.A.dot(x_esti)
        P_pred = self.A.dot(P.dot(self.A.T)) + self.Q
        K = P_pred.dot(self.H.T.dot(inv(self.H.dot(P_pred.dot(self.H.T)) + self.R)))
        x_esti = x_pred + K.dot(z_meas - self.H.dot(x_pred))
        P = P_pred - K.dot(self.H.dot(P_pred))
        return x_esti, P


    def calc_estimated_vel(self):
        for i in range(self.vehicle_num):
            if i != self.my_index - 1:

                x_pose = self.poses[i].pose.pose.position.x
                y_pose = self.poses[i].pose.pose.position.y

                if(self.mean_count[i] < 5):
                    self.z_mean[i][self.mean_count[i]] = [x_pose,y_pose]    

                else:          
                    self.mean_count[i] = 0
                    z_meas = np.mean(self.z_mean[i],axis=0).T
                    self.x_esti[i], self.P[i] = self.kalman_filter(z_meas, self.x_esti[i], self.P[i])
                    self.estimated_vel[i] = [self.x_esti[i][1], self.x_esti[i][3]]
                    # print("x:",self.x_esti[i][0],"/y:",self.x_esti[i][2],"/u:",self.x_esti[i][1],"/v:",self.x_esti[i][3],"/x_pose:",x_pose,"/y_pose:",y_pose)
                self.mean_count[i] += self.mean_count[i] + 1


    #overtake logic
    def overtake_logic(self):
        for i in range(self.vehicle_num):
            if (i != self.my_index - 1):
                path_dx = self.global_path.poses[int(self.poses_idx[0]+2)].pose.position.x - self.global_path.poses[int(self.poses_idx[0])].pose.position.x
                path_dy = self.global_path.poses[int(self.poses_idx[0]+2)].pose.position.y - self.global_path.poses[int(self.poses_idx[0])].pose.position.y
                abspath=(path_dx*path_dx)+(path_dy*path_dy)
                path_vector=np.array([path_dx, path_dy])
                car_dx = self.poses[i].pose.pose.position.x - self.my_pose[0]
                car_dy = self.poses[i].pose.pose.position.y - self.my_pose[1]
                abscar=(car_dx*car_dx)+(car_dy*car_dy)
                car_vector=np.array([car_dx, car_dy])
                car_angle = 180*np.dot(path_vector,car_vector)/(sqrt(abspath)*sqrt(abscar))/pi
                self.car_angle[i]=car_angle   


            if(self.front_or_behind[i]==True):#if we are behind
                if(self.distance[i]<8): #nearby
                    if(abs(self.car_angle[i])< pi/3):   #angle low
                        self.side_enabled[i]=True
                        self.overtake_flag[i]=True #overtake_flag
                        self.front_enabled[i]=True
                    else:                               #angle high
                        self.side_enabled[i]=False
                        self.overtake_flag[i]=True#overtake_flag
                        self.front_enabled[i]=True
                else:   #far
                    self.side_enabled[i]=False
                    self.overtake_flag[i]=False #overtake_flag
                    self.front_enabled[i]=False

            else:   #if we are forward #no overtake
                if(self.distance[i]<8): #nearby
                    if(abs(self.car_angle[i])< pi/3):   #angle low
                        self.side_enabled[i]=False
                        self.overtake_flag[i]=False #overtake_flag
                        self.front_enabled[i]=True
                    else:                               #angle high
                        self.side_enabled[i]=False
                        self.overtake_flag[i]=False #overtake_flag
                        self.front_enabled[i]=True
                else:    #far
                    self.side_enabled[i]=False
                    self.overtake_flag[i]=False #overtake_flag
                    self.front_enabled[i]=False

            x_vel=self.estimated_vel[i][0]
            y_vel=self.estimated_vel[i][1]
            abs_vel=sqrt(x_vel*x_vel+y_vel*y_vel)

            if(abs_vel<1):
                self.side_enabled[i]=False
                self.overtake_flag[i]=False
                self.front_enabled[i]=False



    def apply_reconfigure(self):

        #if the vehicle is the first
        if self.front_or_behind[1] == False:
            if self.distance[1]>8:
                self.front_enabled[1] = False
        if self.front_or_behind[2] == False:
            if self.distance[2]>8:
                self.front_enabled[2] = False

        #Overtake flag
        if self.overtake_flag[1] == True & self.overtake_flag[2] == True:
            self.clients[2].update_configuration({"lookahead_dist":2.5})
            rospy.set_param('/erp42_1/move_base/RegulatedPurePursuitController/lookahead_dist', 2.5)
            self.clients[2].update_configuration({"cost_scaling_dist":2.5})
            rospy.set_param('/erp42_1/move_base/RegulatedPurePursuitController/cost_scaling_dist', 2.5)
            self.clients[2].update_configuration({"cost_scaling_gain":0.2})
            rospy.set_param('/erp42_1/move_base/RegulatedPurePursuitController/cost_scaling_gain', 0.2)
        elif self.overtake_flag[1] == True & self.overtake_flag[2] == False:
            self.clients[2].update_configuration({"lookahead_dist":1.0})
            rospy.set_param('/erp42_1/move_base/RegulatedPurePursuitController/lookahead_dist', 1.0)
            self.clients[2].update_configuration({"cost_scaling_dist":0.5})
            rospy.set_param('/erp42_1/move_base/RegulatedPurePursuitController/cost_scaling_dist', 0.5)
            self.clients[2].update_configuration({"cost_scaling_gain":0.8})
            rospy.set_param('/erp42_1/move_base/RegulatedPurePursuitController/cost_scaling_gain', 0.8)
        elif self.overtake_flag[1] == False & self.overtake_flag[2] == True:
            self.clients[2].update_configuration({"lookahead_dist":1.0})
            rospy.set_param('/erp42_1/move_base/RegulatedPurePursuitController/lookahead_dist', 1.0)
            self.clients[2].update_configuration({"cost_scaling_dist":0.5})
            rospy.set_param('/erp42_1/move_base/RegulatedPurePursuitController/cost_scaling_dist', 0.5)
            self.clients[2].update_configuration({"cost_scaling_gain":0.8})
            rospy.set_param('/erp42_1/move_base/RegulatedPurePursuitController/cost_scaling_gain', 0.8)
        else:
            self.clients[2].update_configuration({"lookahead_dist":2.5})
            rospy.set_param('/erp42_1/move_base/RegulatedPurePursuitController/lookahead_dist', 2.5)
            self.clients[2].update_configuration({"cost_scaling_dist":2.5})
            rospy.set_param('/erp42_1/move_base/RegulatedPurePursuitController/cost_scaling_dist', 2.5)
            self.clients[2].update_configuration({"cost_scaling_gain":0.2})
            rospy.set_param('/erp42_1/move_base/RegulatedPurePursuitController/cost_scaling_gain', 0.2)


        if self.side_enabled[1] == True:
            self.clients[0].update_configuration({"enabled":True})
            rospy.set_param('/erp42_1/move_base/global_costmap/overtake_layer_2/enabled', True)
        else:
            self.clients[0].update_configuration({"enabled":False})
            rospy.set_param('/erp42_1/move_base/global_costmap/overtake_layer_2/enabled', False)

        # if self.side_enabled[2] == True:
        #     self.clients[1].update_configuration({"enabled":True})
        #     rospy.set_param('/erp42_1/move_base/global_costmap/overtake_layer_3/enabled', True)
        # else:
        #     self.clients[1].update_configuration({"enabled":False})
        #     rospy.set_param('/erp42_1/move_base/global_costmap/overtake_layer_3/enabled', False)
        
        if self.front_enabled[1] == True:
            self.clients[1].update_configuration({"enabled":True})
            rospy.set_param('/erp42_1/move_base/global_costmap/path_layer_2/enabled', True)
        else:
            self.clients[1].update_configuration({"enabled":False})
            rospy.set_param('/erp42_1/move_base/global_costmap/path_layer_2/enabled', False)
        
        # if self.front_enabled[2] == True:
        #     self.clients[3].update_configuration({"enabled":True})
        #     rospy.set_param('/erp42_1/move_base/global_costmap/path_layer_3/enabled', True)
        # else:
        #     self.clients[3].update_configuration({"enabled":False})
        #     rospy.set_param('/erp42_1/move_base/global_costmap/path_layer_3/enabled', False)

        if self.right_or_left[1] == True:
            self.clients[0].update_configuration({"right_or_left":True})
            rospy.set_param('/erp42_1/move_base/global_costmap/overtake_layer_2/right_or_left', True)
        else:
            self.clients[0].update_configuration({"right_or_left":False})
            rospy.set_param('/erp42_1/move_base/global_costmap/overtake_layer_2/right_or_left', False)
        
        if self.right_or_left[2] == True:
            self.clients[1].update_configuration({"right_or_left":True})
            rospy.set_param('/erp42_1/move_base/global_costmap/overtake_layer_3/right_or_left', True)
        else:
            self.clients[1].update_configuration({"right_or_left":False})
            rospy.set_param('/erp42_1/move_base/global_costmap/overtake_layer_3/right_or_left', False)

        self.clients[1].update_configuration({"path_vector":self.path_vector[1]})
        rospy.set_param('/erp42_1/move_base/global_costmap/path_layer_2/path_vector', float(self.path_vector[1]))

        self.clients[3].update_configuration({"path_vector":self.path_vector[2]})
        rospy.set_param('/erp42_1/move_base/global_costmap/path_layer_3/path_vector', float(self.path_vector[2]))
        
        print("erp42_2\tside_enabled : {0}\n\terp42_2\tfront_enabled : {1}\n\tright_or_left : {2}\n\tfront_or_behind : {3}\n\tdistance : {4}\n\tovertake_flag : {5}\n\testimated_vel_x : {6}\n\testimated_vel_y : {7}\n\tcar_angle: : {8}\n"
            .format(self.side_enabled[1], self.front_enabled[1], self.right_or_left[1], self.front_or_behind[1], self.distance[1], self.overtake_flag[1], self.estimated_vel[1][0], self.estimated_vel[1][1], self.car_angle[1]))
        print("erp42_3\tside_enabled : {0}\n\terp42_2\tfront_enabled : {1}\n\tright_or_left : {2}\n\tfront_or_behind : {3}\n\tdistance : {4}\n\tovertake_flag : {5}\n\testimated_vel_x : {6}\n\testimated_vel_y : {7}\n\tcar_angle: : {8}\n"
            .format(self.side_enabled[2], self.front_enabled[2], self.right_or_left[2], self.front_or_behind[2], self.distance[2], self.overtake_flag[2], self.estimated_vel[2][0], self.estimated_vel[2][1], self.car_angle[2]))
        print("-----------------------------------------\n")


    def configure(self):
        #Get ROS Parameters
        self.path_file_name = rospy.get_param("~path_file_name")
        self.path_frame = rospy.get_param("~path_frame")
        self.vehicle_num = int(rospy.get_param("~vehicle_num"))
        self.frequency = rospy.get_param("~frequency")
        self.my_index = rospy.get_param("~my_index")
        self.overtake_dist = rospy.get_param("~overtake_dist")

        #Initialize variables
        self.rate = rospy.Rate(self.frequency)
        self.my_pose = np.zeros(3) # [position.x, position.y, orientation vector]
        self.poses = np.zeros(self.vehicle_num, dtype=Odometry)
        self.poses_idx = np.zeros(self.vehicle_num)
        self.prev_poses_idx = np.zeros(self.vehicle_num,dtype=int)
        self.initial_check = np.full((self.vehicle_num,), False)
        self.initial_check_pose = False

        #Overtake layer parameters
        self.distance = np.zeros(self.vehicle_num)
        self.in_distance_counter = np.zeros(self.vehicle_num)
        self.right_or_left = np.full((self.vehicle_num,), False)
        self.front_enabled = np.full((self.vehicle_num,), False)
        self.side_enabled = np.full((self.vehicle_num,),False)
        self.front_or_behind = np.full((self.vehicle_num,), False)
        # self.estimated_pose = np.zeros(self.vehicle_num)
        self.overtake_flag=np.full((self.vehicle_num,), False)
        self.car_angle=np.zeros(self.vehicle_num)
        self.estimated_vel = np.zeros((self.vehicle_num, 2))
        self.path_vector = np.zeros(self.vehicle_num)

        self.client_overtake_2 = dynamic_reconfigure.client.Client("erp42_1/move_base/global_costmap/overtake_layer_2", timeout=100, config_callback=self.callback)
        #self.client_overtake_3 = dynamic_reconfigure.client.Client("erp42_1/move_base/global_costmap/overtake_layer_3", timeout=100, config_callback=self.callback)
        self.client_path_2 = dynamic_reconfigure.client.Client("erp42_1/move_base/global_costmap/path_layer_2", timeout=100, config_callback=self.callback)
        #self.client_path_3 = dynamic_reconfigure.client.Client("erp42_1/move_base/global_costmap/path_layer_3", timeout=100, config_callback=self.callback)
        self.client_planner = dynamic_reconfigure.client.Client("erp42_1/move_base/RegulatedPurePursuitController",timeout=100, config_callback=self.callback)
        #self.clients = [self.client_overtake_2, self.client_overtake_3, self.client_path_2, self.client_path_3, self.client_planner]
        self.clients = [self.client_overtake_2, self.client_path_2, self.client_planner]

        #Kalman_filter
        self.dt = 0.05
        self.A = np.array([[1, self.dt, 0 ,0],
                            [0, 1, 0 ,0],
                            [0, 0, 1, self.dt],
                            [0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0],
                            [0, 0, 1, 0]])
        self.Q = 10 * np.eye(4)
        self.R = np.array([[100]])
        
        self.z_mean = np.full((self.vehicle_num, 5, 2), np.zeros((5,2)))
        self.mean_count = np.zeros(self.vehicle_num, dtype=np.int)
        self.P_0 = np.full((self.vehicle_num, 4, 4), np.array([[  23.55254745,   19.55088905,   23.55254745,   19.55088905],
                            [  19.55088905,  222.99915521,   19.55088905,   17.93666497],
                            [  23.55254745,   19.55088905,   23.55254745,   19.55088905],
                            [  19.55088905,   17.93666497,   19.55088905,  222.99915521]]))
        self.x_0 = np.full((self.vehicle_num, 4), 0.0)
        self.x_esti, self.P = self.x_0, self.P_0


    def callback(self, config):
        return config

if __name__ == '__main__':
    ParamReconfig()