#!/usr/bin/env python3
import math

import numpy as np
import rospy
import time
from std_msgs.msg import Float64MultiArray, Float64
from modules.sprites import *

rospy.init_node('pure_pursuit_control')
long_drive_pub = rospy.Publisher('controls/throttle', Float64, queue_size=10)
steering_drive_pub = rospy.Publisher('controls/steer', Float64, queue_size=10)

current_state = np.zeros(6)


def update_car_state(data):
    global current_state
    current_state = np.array(data.data)  # [x, y, theta, speed, beta (slip angle), theta_dot]


curr_waypoint = None


def update_waypoint(data):
    global curr_waypoint
    curr_waypoint = np.array(data.data)  # [x, y, yaw_path] of next waypoint


class Controller:
    esum = 0
    eprev = 0

    def __init__(self, L=4.9):
        self.L = L

    def get_longitudinal_control(self, v_current, v_desired, dt):
        Kp = 0.5
        Ki = 0.1
        Kd = 0.2
        e = v_desired - v_current
        self.esum += e
        deltaEdeltaT = (e - self.eprev) / dt
        self.eprev = e
        action = Kp * e + Ki * self.esum + Kd * deltaEdeltaT
        return float(math.tanh(action))
        '''
        PID Longitudinal controller
        Parameters
        ----------
        v_current: float
            Current speed of the vehicle
        v_desired: float
            Desired speed of the vehicle
        dt: float
            Delta time since last time the function was called

        Returns
        -------
        throttle_output: float
            Value in the range [-1,1]
        '''
        pass

    def get_lateral_pure_pursuit(self, current_xy, current_yaw, next_waypoint):
        Id = math.sqrt(math.pow(current_xy[0] - next_waypoint[0], 2) + math.pow(current_xy[1] - next_waypoint[1], 2))
        alpha = math.atan2(next_waypoint[1] - current_xy[1], next_waypoint[0] - current_xy[0]) - current_yaw
        e = Id * math.sin(alpha)
        return float(math.atan2(2 * self.L * math.sin(alpha), Id))

        '''
        Pure Pursuit, lateral controller

        Parameters
        ----------
        current_xy: np.array of floats, shape=2
            Current position of the vehicle [x,y] given in CG frame
        current_yaw: float
            Current heading of the vehicle
        next_waypoint: np.array of floats, shape=2
            Next waypoint for the vehicle to reach [x,y]

        Returns
        -------
        steer_angle: float
            Steering angle in rad
        '''
        pass

    def get_lateral_stanley(self, current_xy, current_yaw, current_speed, next_waypoint):
        Id = math.sqrt(math.pow(current_xy[0] - next_waypoint[0], 2) + math.pow(current_xy[1] - next_waypoint[1], 2))
        psi = next_waypoint[2] - current_yaw
        e = Id * math.sin(psi)
        return psi + math.atan2(0.5 * e, current_speed + 0.5)
        '''
        Stanley, lateral controller

        Parameters
        ----------
        current_xy: np.array of floats, shape=2
            Current position of the vehicle [x,y] given in CG frame
        current_yaw: float
            Current heading of the vehicle
        current_speed: float
            Current speed of the vehicle
        next_waypoint: np.array of floats, shape=3
            Next waypoint for the vehicle to reach [x,y,yaw_path]

        Returns
        -------
        steer_angle: float
            Steering angle in rad
        '''
        pass


controller = Controller()

rate = 10
r = rospy.Rate(rate)

while not rospy.is_shutdown():
    r.sleep()
    if curr_waypoint is None:
        continue

    # Getting states variables from current car state (position, heading, speed)
    rospy.Subscriber("vehicle_model/state", Float64MultiArray, update_car_state)
    rospy.Subscriber("waypoints", Float64MultiArray, update_waypoint)

    # Longitudinal and lateral control
    longitudinal_cont = controller.get_longitudinal_control(current_state[3], 15,1)
    lateral_cont = controller.get_lateral_pure_pursuit(np.array(current_state[0], current_state[1]), current_state[2],
                                                       current_state[3], np.array(curr_waypoint[0], curr_waypoint[1]))

    # Create longitudinal and lateral messages (of type Float64 imported above)
    longitudinal_cont = Float64(longitudinal_cont)
    lateral_cont = Float64(lateral_cont)

    # Publish the 2 messages
    long_drive_pub.publish(longitudinal_cont)
    steering_drive_pub.publish(lateral_cont)
    print("Torque: {:.2f}, Steering angle: {:.2f}".format(longitudinal_cont, lateral_cont))
