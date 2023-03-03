#!/usr/bin/env python3
import numpy as np
import rospy
from modules.tires import *
from modules.vehicle_models import *
from modules.simulator import *
from modules.engine_models import *
from modules.steering_models import *

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

rospy.init_node('vehicle_model')
vehicle_state_pub = rospy.Publisher('vehicle_model/state', Float64MultiArray, queue_size=10) 
current_state = Float64MultiArray()
layout = MultiArrayLayout()
dimension = MultiArrayDimension()
dimension.label = "current_state"
dimension.size = 6
dimension.stride = 6
layout.data_offset = 0
layout.dim = [dimension]



# Create car simulator/physics engine using the parameters in the launch file
tire = TirePacejka() # Has 14 parameters set to default values
engine_f = EngineSimple(c_m=0.8,c_r_0=0,c_r_2=5)
engine_r = EngineSimple(c_m=0.8,c_r_0=0,c_r_2=5)
steering = SteeringLinear(min_angle=-27.2,max_angle=27.2)
vehicle = VehicleSimpleNonlinear(tire,engine_f,engine_r,steering)
vehicle.mF0 = 500 # Mass on front axle  [kg]
vehicle.mR0 = 700 # Mass on rear axle [kg]
vehicle.IT = 10000 # Moment of intertia of the car [kg*m2]
vehicle.lT = 4.9 # Car length (wheel base) [m]
vehicle.wT = 1.9 # Car width [m]
vehicle.muy = 3 # Operational friction coefficient
sim = Simulator(vehicle,initial_state=[0,0,0.5,1,0,0])
dt = 0.1
longitudinal_multiplier = 10000

def drive(data):
    sim.vehicle.AI2VCU_Drive_F(data.data*longitudinal_multiplier,0)
    sim.vehicle.AI2VCU_Drive_R(data.data*longitudinal_multiplier,0)

def steer(data):
    sim.vehicle.AI2VCU_Steer(data.data*180/np.pi)
    
rospy.Subscriber("controls/throttle",Float64,drive)
rospy.Subscriber("controls/steer",Float64,steer)


r = rospy.Rate(10)

while not rospy.is_shutdown():
    sim.step(step_size=dt)
    
    
    current_state.data = sim.current_state
    current_state.layout = layout
    
    vehicle_state_pub.publish(current_state)
    r.sleep()
