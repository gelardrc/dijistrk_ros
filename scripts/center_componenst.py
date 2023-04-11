#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from gazebo_msgs.srv import SpawnModel,DeleteModel,SetModelState,GetModelState,GetWorldProperties
from geometry_msgs.msg import Pose,Point,Quaternion,Wrench,Vector3
from gazebo_msgs.msg import ModelState




rospy.init_node('teste')

rospy.wait_for_service('/gazebo/spawn_sdf_model')
        
rospy.wait_for_service('/gazebo/get_model_state')
        
rospy.wait_for_service('/gazebo/set_model_state')

get_componests = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)

set_component = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)

get_model = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)

teste = get_componests()

names = [   'aws_robomaker_warehouse_ShelfF_01_001', 
            'aws_robomaker_warehouse_ShelfE_01_001', 
            'aws_robomaker_warehouse_ShelfE_01_002',
            'aws_robomaker_warehouse_ShelfE_01_003', 
            'aws_robomaker_warehouse_ShelfD_01_001', 
            'aws_robomaker_warehouse_ShelfD_01_002',
            'aws_robomaker_warehouse_ShelfD_01_003', 
            'aws_robomaker_warehouse_GroundB_01_001', 
            'aws_robomaker_warehouse_Lamp_01_005',
            'aws_robomaker_warehouse_Bucket_01_020', 
            'aws_robomaker_warehouse_Bucket_01_021', 
            'aws_robomaker_warehouse_Bucket_01_022',
            'aws_robomaker_warehouse_ClutteringA_01_016', 
            'aws_robomaker_warehouse_ClutteringA_01_017',
            'aws_robomaker_warehouse_ClutteringA_01_018',
            'aws_robomaker_warehouse_ClutteringC_01_027',
            'aws_robomaker_warehouse_ClutteringC_01_028', 
            'aws_robomaker_warehouse_ClutteringC_01_029',
            'aws_robomaker_warehouse_ClutteringC_01_030', 
            'aws_robomaker_warehouse_ClutteringC_01_031',
            'aws_robomaker_warehouse_ClutteringC_01_032', 
            'aws_robomaker_warehouse_ClutteringD_01_005',
            'aws_robomaker_warehouse_TrashCanC_01_002', 
            'aws_robomaker_warehouse_PalletJackB_01_001']


for i in names:
    print('model',i)
    t = get_model(i,'world')
    new_pose = Pose()   

    new_pose.position.x = t.pose.position.x + 7
    new_pose.position.y = t.pose.position.y + 10
    new_pose.position.z = t.pose.position.z

    new_pose.orientation = t.pose.orientation

    set_component(ModelState(
            model_name = i,
            pose = new_pose
        ))   


while not (rospy.is_shutdown):
    
    continue

