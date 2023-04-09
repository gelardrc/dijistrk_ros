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

get_componests = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)

set_component = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)

get_model = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)

teste = get_componests()

for i in teste.model_names:
    t = get_model(i,'world')
    new_pose = Pose()   

    new_pose.position.x = t.position.x + 7
    new_pose.position.y = t.position.y + 10
    new_pose.position.z = t.position.z + 0

    new_pose.orientation = t.orientation

    set_component(ModelState(
            model_name = i,
            pose = new_pose
        ))   


while not (rospy.is_shutdown):
    
    continue

