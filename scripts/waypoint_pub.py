#!/usr/bin/env python3
import numpy as np
import rospy

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from modules.sprites import *

rospy.init_node('waypoints_node')
waypoints_pub = rospy.Publisher('waypoints', Float64MultiArray, queue_size=10)

current_state = np.zeros(6)
def update_car_state(data):
    global current_state
    current_state = np.array(data.data)
    

all_waypoints = None
waypoint_idx = 0

def create_msg(value, name="message"):
    command = Float64MultiArray()
    layout = MultiArrayLayout()
    dimension = MultiArrayDimension()
    dimension.label = name
    dimension.size = len(value)
    dimension.stride = len(value)
    layout.data_offset = 0
    layout.dim = [dimension]
    command.layout = layout
    command.data = value
    return command

def parse_paths(path_descs):
    paths = []
    current_path = []
    path_length = 0
    for i in range(len(path_descs)):
        if path_length == 0:
            path_length = path_descs[i]
            if len(current_path) > 0:
                paths.append(current_path)
                current_path = []
        else:
            current_path.append(path_descs[i])
            path_length -= 1
    if len(current_path) > 0:
        paths.append(current_path)
    path_objs = []
    for path in paths:
        spline_degree =  int(path[0])
        smoothness = path[1]
        resolution = int(path[2])
        color = (int(path[3]),int(path[4]),int(path[5]))
        width = int(path[6])
        path_objs.append(Path(np.array(path[7:]).reshape(-1,2),np.array([1200,700]),
                          spline_degree=spline_degree,
                          smoothness=smoothness,
                          resolution=resolution,
                          color=color,
                          width=width))
    return path_objs
    
def update_paths(data):
    global all_waypoints
    path_objs = parse_paths(data.data)
    all_waypoints = path_objs[0].smooth_points # Choose the first path to follow


rospy.Subscriber("vehicle_model/state",Float64MultiArray,update_car_state)
rospy.Subscriber("paths/current_paths",Float64MultiArray,update_paths)

waypoint_achieved_threshold = 5

r = rospy.Rate(10)

while not rospy.is_shutdown():
    r.sleep()
    if all_waypoints is None:
        continue

    current_position = current_state[:2]
    
    # Moving to next waypoint if reached one
    if np.linalg.norm(current_position-all_waypoints[waypoint_idx]) < waypoint_achieved_threshold:
        waypoint_idx = (waypoint_idx+1)%all_waypoints.shape[0]
        print("Moved waypoint to x={:.1f},y={:.1f}".format(all_waypoints[waypoint_idx][0],all_waypoints[waypoint_idx][1]))

    # Sending logitudinal and lateral commands
    waypoint = all_waypoints[waypoint_idx]
    next_waypoint_idx = (waypoint_idx+1)%all_waypoints.shape[0]
    next_waypoint = all_waypoints[next_waypoint_idx]
    diff = next_waypoint - waypoint
    yaw_path = np.arctan2(diff[1], diff[0])
    
    waypoints_msg = create_msg([waypoint[0], waypoint[1], yaw_path], "Next Waypoint")
    waypoints_pub.publish(waypoints_msg)