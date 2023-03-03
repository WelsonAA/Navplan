#!/usr/bin/env python3
import numpy as np

class EngineSimple:
    def __init__(self,c_m=0.8,c_r_0=2000,c_r_2=0.4):
        self.c_m = c_m
        self.c_r_0 = c_r_0
        self.c_r_2 = c_r_2
        self.Fx_without_drag = 0
        
    def command(self,torque_request,max_rpm):
        self.Fx_without_drag = self.c_m*torque_request - self.c_r_0
    
    def get_long_force(self,t,states):
        sign = 1
        if states[3] < 0:
            sign = -1
        Fx = self.Fx_without_drag - sign*self.c_r_2*states[3]**2
        return Fx
        
    
class EngineSimpleExp:
    def __init__(self):
        pass
    
    def command(self,torque_request,max_rpm):
        pass
    
    def get_long_force(self,t,states):
        pass