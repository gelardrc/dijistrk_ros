#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from tf.transformations import quaternion_from_euler
import numpy as np
import heapq
import math
start = None
goal  = None
field = None
origin= None
resolution= None

class Node:
    def __init__(self, x, y, cost=float('inf')):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = None

# Directions for 8-way movement
DIRECTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

def is_valid(x, y, field):
    return 0 <= x < field.shape[0] and 0 <= y < field.shape[1] and field[x, y] == 0

def dijkstra(field, start, goal):
    start_node = Node(start[0], start[1], cost=0)
    pq = []
    heapq.heappush(pq, (start_node.cost, start_node))
    visited = {}

    while pq:
        current_cost, current_node = heapq.heappop(pq)
        current_pos = (current_node.x, current_node.y)

        if current_pos in visited:
            continue

        visited[current_pos] = current_node

        if current_pos == (goal[0], goal[1]):
            return reconstruct_path(current_node)

        for dx, dy in DIRECTIONS:
            nx, ny = current_node.x + dx, current_node.y + dy
            if is_valid(nx, ny, field) and (nx, ny) not in visited:
                new_cost = current_cost + math.sqrt(dx**2 + dy**2)
                neighbor = Node(nx, ny, new_cost)
                neighbor.parent = current_node
                heapq.heappush(pq, (new_cost, neighbor))

    return []  # No path found

def reconstruct_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

def calculate_orientation(x1, y1, x2, y2):
    """Calcula a orientação (em radianos) entre dois pontos"""
    dx = x2 - x1
    dy = y2 - y1
    angle = math.atan2(dy, dx)  # Calcula o ângulo entre os dois pontos
    return angle

def quaternion_from_yaw(yaw):
    """Converte um ângulo (yaw) em um quaternion"""
    q = quaternion_from_euler(0, 0, yaw)
    return q

def send_path(path, origin):
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()

    resolution = 1
    for i,(x, y) in enumerate(path):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = y * resolution + origin.y
        pose.pose.position.y = x * resolution + origin.x
        pose.pose.position.z = 0
        
        if i < len(path) - 1:
            next_x, next_y = path[i + 1]
            yaw = calculate_orientation(y, x, next_y, next_x)
            quaternion = quaternion_from_yaw(yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
        else:
            # No último waypoint, manter a orientação anterior ou definir sem rotação
            pose.pose.orientation.w = 1.0  # Sem rotação

        path_msg.poses.append(pose)

    if path_msg.poses:
        rospy.loginfo_once("Publishing path with {} waypoints...".format(len(path_msg.poses)))
        path_pub.publish(path_msg)
    else:
        rospy.logwarn("Path is empty. Nothing to publish.")

def start_cb(data):
    global start,origin,resolution
    start = [data.pose.pose.position.y,data.pose.pose.position.x]
    start[0] = int((start[0] - origin.y)/resolution)
    start[1] = int((start[1] - origin.x)/resolution)
    cal_path()

def goal_cb(data):
    global goal
    goal = [data.pose.position.y,data.pose.position.x]
    goal[0] = int((goal[0] - origin.y)/resolution)
    goal[1] = int((goal[1] - origin.x)/resolution)
    cal_path()

def map_callback(msg):
    global start,goal,field,resolution,origin

    rospy.loginfo("Loading map...")
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin = msg.info.origin.position

    field = np.array(msg.data).reshape((height, width))
    field = np.where(field != 0, 1, 0)

    rospy.loginfo("Finding path...")
    
    default_start = (33, 33)
    default_goal = (33, 44)
    
    name = rospy.get_name()
    if bool(rospy.get_param(name+"/choose",False)):
        rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,start_cb)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_cb)
    else:    
        start = eval(rospy.get_param(name+"/start",default_start))
        goal =  eval(rospy.get_param(name+"/goal",default_goal))
        for i in range(20):
            cal_path()
        

def cal_path():
    global field,start,goal,origin
    if start and goal and field is not None:
        rospy.loginfo_once("Start:{0} Goal:{1}".format(start,goal))
        path = dijkstra(field, start, goal)
        if path:
            rospy.loginfo_once("Found path: {}".format(path))
            send_path(path, origin)
        else:
            rospy.logwarn_once("No path found.")


if __name__ == "__main__":
    rospy.init_node("dijkstra_planner", anonymous=False)
    path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.spin()
