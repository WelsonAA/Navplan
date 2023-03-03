#!/usr/bin/env python3
import numpy as np

class SteeringSimple:
    '''
    Simple steering model that can be extended upon later.
    It takes as input the steering_angle_request and adjusts the steering accordingly using "command" function
    The current steering angle can be return using "get_steering_angle"
    This model simply clips the steering angle between minimum and maximum steering angles
    '''
    def __init__(self,min_angle=-27.2,max_angle=27.2):
        self.steering_angle = 0
        self.min_angle = min_angle*np.pi/180
        self.max_angle = max_angle*np.pi/180
    
    def command(self,steering_angle_request):
        self.steering_angle = np.clip(steering_angle_request,self.min_angle,self.max_angle)
        
    def get_steering_angle(self,t,states):
        return self.steering_angle
    
class SteeringLinear:
    '''
    Linear steering model, changes the steering angle linearly in time and clips angle between the minimum and maximum angle
    It takes as input the steering_angle_request and adjusts the steering accordingly using "command" function
    The current steering angle can be return using "get_steering_angle"
    '''
    def __init__(self,min_angle=-27.2,max_angle=27.2,steering_speed=2):
        self.steering_angle = 0
        self.min_angle = min_angle*np.pi/180
        self.max_angle = max_angle*np.pi/180
        self.last_time_before_change = 0
        self.steering_speed = steering_speed
        self.steering_angle_request = 0
    
    def command(self,steering_angle_request):
        self.steering_angle_request = steering_angle_request
        
    def get_steering_angle(self,t,states):
        steering_angle_possible_diff = self.steering_speed*(t-self.last_time_before_change)
        steering_angle_diff = self.steering_angle_request-self.steering_angle
        
        if steering_angle_possible_diff > np.abs(steering_angle_diff):
            self.steering_angle = self.steering_angle_request
        elif steering_angle_diff > 0:
            self.steering_angle += steering_angle_possible_diff
        else:
            self.steering_angle -= steering_angle_possible_diff
            
        self.steering_angle = np.clip(self.steering_angle,self.min_angle,self.max_angle)
        self.last_time_before_change = t
        return self.steering_angle