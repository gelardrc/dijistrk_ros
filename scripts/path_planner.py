#!/usr/bin/env python

import rospy

from gazebo_msgs.srv import SpawnModel,DeleteModel,SetModelState,GetModelState
from geometry_msgs.msg import Pose,Point,Quaternion,Wrench,Vector3
from gazebo_msgs.msg import ModelState


class world():
    
    def __init__(self):
        
        self.init_ros_services()
        
        pass
    
    def init_ros_services(self):


        pass



def planner():
    
    while not (rospy.is_shutdown()):
    
        continue 
    
    pass


if __name__ == "__main__":
    rospy.init_node("path_planer",anonymous=False)
    planner()
