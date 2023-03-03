#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
import numpy as np

from modules.tires import *
from modules.vehicle_models import *
from modules.simulator import *
from modules.engine_models import *
from modules.steering_models import *
from modules.environment import *
from modules.sprites import *


    
env = Environment(0.1,theme='dark')

###########################################################################################
########################## Add paths and cones here #######################################
###########################################################################################

env.add_path(points=[(0,0),(50,50),(100,0),(50,-40),(25,-15),(0,0)],spline_degree=2,resolution=300)

###########################################################################################
###########################################################################################

def update_car_state(data):
    env.update_state(data.data)

rospy.init_node("environment")
rospy.Subscriber("vehicle_model/state",Float64MultiArray,update_car_state)
paths_publisher = rospy.Publisher('paths/current_paths', Float64MultiArray, queue_size=10) 
cones_publisher = rospy.Publisher('cones/current_cones', Float64MultiArray, queue_size=10) 

# Creating a placeholder message to send current paths
current_paths_msg = Float64MultiArray()
path_layout = MultiArrayLayout()
path_dimension = MultiArrayDimension()
path_dimension.label = "paths_descriptions"
path_dimension.size = np.array(env.path_params).reshape(-1).shape[0]
path_dimension.stride = np.array(env.path_params).reshape(-1).shape[0]
path_layout.data_offset = 0
path_layout.dim = [path_dimension]

# Creating a placeholder message to send current cones
current_cones_msg = Float64MultiArray()
cone_layout = MultiArrayLayout()
cone_dimension = MultiArrayDimension()
cone_dimension.label = "paths_descriptions"
cone_dimension.size = np.array(env.cone_params).reshape(-1).shape[0]
cone_dimension.stride = np.array(env.cone_params).reshape(-1).shape[0]
cone_layout.data_offset = 0
cone_layout.dim = [cone_dimension]

def publish_paths():
    current_paths_msg.data = np.array(env.path_params).reshape(-1).tolist()
    current_paths_msg.layout = path_layout
    
    paths_publisher.publish(current_paths_msg)
    
def publish_cones():
    current_cones_msg.data = np.array(env.cone_params).reshape(-1).tolist()
    current_cones_msg.layout = cone_layout
    
    cones_publisher.publish(current_cones_msg)
    
env.play(funcs_call=[publish_paths,publish_cones])
