#!/usr/bin/env python2.7

import rospy

from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel

class ERP42Manager():
    def __init__(self):
        
        # rospy.init()

        # Configuration
        self._configure()

        # Service clients
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    def _configure(self):
        path_model = None # read from yaml file
        pose_list = None # read from csv file

        # Reset vehicle info
        num_vehicle = 0
        num_vehicle_preset = None # read from yaml file

    def spin(self):
        # Get keyboard input
        pass

        # Call service by keyboard input
        pass

    def _spawn_vehicle(self):
        rospy.wait_for_service("/gazebo/spawn_urdf_model")

        try:
            spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
            spawn_model_client(
                model_name='ground_plane',
                model_xml=open(path_model, 'r').read(),
                robot_namespace='/erp42_01', # TODO;
                initial_pose=Pose(pose_list(0)),
                reference_frame='world'
            )

        except rospy.ServiceException as e:
            print("Service call failed: ",e)
        
    def _spawn_preset(self):
        pass
        
    def _despawn_vehicle(self):
        pass
    
    def _despawn_all(self):
        pass
    
    def _reset_vehicles(self):
        pass

if __name__ == '__main__':
    manager = ERP42Manager()
    manager.spin()