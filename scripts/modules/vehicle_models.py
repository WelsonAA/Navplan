#!/usr/bin/env python3
import numpy as np

class VehicleSimple:
    def __init__(self):
        self.mT = None       # Mass of the car (tractor) [kg]
        self.IT = None       # Moment of inertia the car (tractor) [kg * m2]
        self.a = None        # Distance from front axle of the car (tractor) to the center of mass of the car (tractor) [m]
        self.b = None        # Distance from center of mass of the car (tractor) to the front axle of the car (tractor) [m]
        self.mF0 = None      # Mass over the front axle [kg]
        self.mR0 = None      # Mass over the rear axle [kg]
        self.lT = None       # Wheelbase [m]
        self.nF = None       # Number of front tires
        self.nR = None       # Number of rear tires
        self.wT = None       # Track of the car (tractor)  [m]
        self.muy = None      # Operational friction coefficient
        self.tire = None     # Tire model
        self.engine_f = None # Front Engine Model
        self.engine_r = None # Rear Engine Model
        self.steering = None # Steering Model
        self.num_state_vars = None # Number of state variables for any vehicle model
        
    def model(self,t,states):
        pass
    
    def get_mT(self):
        return self.mF0 + self.mR0
    
    def get_a(self):
        return self.mR0 / self.get_mT() * self.lT
    
    def get_b(self):
        return self.lT - self.get_a()
    
    def AI2VCU_Drive_F(self,FRONT_AXLE_TRQ_REQUEST_Nm,FRONT_MOTOR_SPEED_MAX_rpm):
        self.engine_f.command(FRONT_AXLE_TRQ_REQUEST_Nm,FRONT_MOTOR_SPEED_MAX_rpm)
    
    def AI2VCU_Drive_R(self,REAR_AXLE_TRQ_REQUEST_Nm,REAR_MOTOR_SPEED_MAX_rpm):
        self.engine_r.command(REAR_AXLE_TRQ_REQUEST_Nm,REAR_MOTOR_SPEED_MAX_rpm)
    
    def AI2VCU_Steer(self,STEER_REQUEST_deg):
        self.steering.command(STEER_REQUEST_deg)
    
class VehicleSimpleLinear(VehicleSimple):
    def __init__(self,tire_model,engine_model_f,engine_model_r,steering_model):
        self.mF0 = 700  # Mass on front axle  [kg]
        self.mR0 = 600  # Mass on rear axle [kg]
        self.IT = 10000 # Moment of intertia of the car [kg*m2]
        self.lT = 3.5   # Car length (wheel base) [m]
        self.nF = 2     # Number of wheels on front axle
        self.nR = 2     # Number of wheels on rear axle
        self.wT = 2     # Car width [m]
        self.muy = 0.8  # Operational friction coefficient
        self.tire = tire_model          # Car Tire Model
        self.engine_f = engine_model_f  # Car front axle engine model
        self.engine_r = engine_model_r  # Car rear axle engine model
        self.steering = steering_model  # Car stering model
        self.num_state_vars = 6         # Number of state variables for the vehicle model
        
    def model(self,t,states):
        # Getting some variables as local variables for simpler equations
        mT = self.get_mT() # Mass of the car [kg]
        IT = self.IT
        a = self.get_a()
        b = self.get_b()
        nF = self.nF
        nR = self.nR
        muy = self.muy
        g = 9.81                  # Gravity [m/s^2]
        v0 = 20                   # [m/s]

        FzF = self.mF0 * g        # Vertical load @ F [N]
        FzR = self.mR0 * g        # Vertical load @ R [N]

        # State variables
        X = states[0]              # X position [m], not used in this model
        Y = states[1]              # Y position [m], not used in this model
        PSI = states[2]            # Yaw angle [rad]
        v = states[3]              # Velocity [m/s]
        ALPHAT = states[4]         # Slip angle [rad]
        dPSI = states[5]           # Yaw rate [rad/s]

        # Get controls
        deltaf = self.steering.get_steering_angle(t,states) # Current steering angle
        FxF = self.engine_f.get_long_force(t,states)        # Current force at front axle
        FxR = self.engine_r.get_long_force(t,states)        # Current force at rear axle
            
        # Slip angles
        ALPHAF = ALPHAT + a/v0*dPSI - deltaf
        ALPHAR = ALPHAT - b/v0*dPSI
            
        # Lateral force
        FyF = nF * self.tire.characteristic(ALPHAF, FzF / nF, muy)
        FyR = nR * self.tire.characteristic(ALPHAR, FzR / nR, muy)

        # State equations
        dx = np.zeros(6)
        dx[0] = VT
        dx[1] = v0*(PSI + ALPHAT)
        dx[2] = dPSI
        dx[3] = (FxF + FxR)/mT
        dx[4] = (FyF + FyR)/(mT*v0) - dPSI
        dx[5] = (a*FyF - b*FyR)/IT
        return dx
    
class VehicleSimpleNonlinear(VehicleSimple):
    def __init__(self,tire_model,engine_model_f,engine_model_r,steering_model):
        self.mF0 = 700  # Mass on front axle  [kg]
        self.mR0 = 600  # Mass on rear axle [kg]
        self.IT = 10000 # Moment of intertia of the car [kg*m2]
        self.lT = 3.5   # Car length (wheel base) [m]
        self.nF = 2     # Number of wheels on front axle
        self.nR = 2     # Number of wheels on rear axle
        self.wT = 2     # Car width [m]
        self.muy = 0.8  # Operational friction coefficient
        self.tire = tire_model          # Car Tire Model
        self.engine_f = engine_model_f  # Car front axle engine model
        self.engine_r = engine_model_r  # Car rear axle engine model
        self.steering = steering_model  # Car stering model
        self.num_state_vars = 6         # Number of state variables for the vehicle model
        self.lateral_controller = None
        self.longitudinal_controller_f = None
        self.longitudinal_controller_r = None
        
    def model(self,t,states):
        if self.lateral_controller is not None:
            self.AI2VCU_Steer(self.lateral_controller.control(t,states))
            
        if self.longitudinal_controller_r is not None:
            self.simulator.vehicle.AI2VCU_Drive_R(*self.longitudinal_controller_r.control(t,states))
            
        if self.longitudinal_controller_f is not None:
            self.simulator.vehicle.AI2VCU_Drive_F(*self.longitudinal_controller_f.control(t,states))
            
        # Getting some variables as local variables for simpler equations
        m = self.get_mT()  # Mass of the car [kg]
        I = self.IT        # Moment of intertia of the car [kg*m2]
        a = self.get_a()
        b = self.get_b()
        nF = self.nF       # Number of wheels on front axle
        nR = self.nR       # Number of wheels on rear axle
        muy = self.muy     # Operational friction coefficient
        g = 9.81           # Gravity [m/s^2]

        FzF = self.mF0 * g # Vertical load at front axle [N]
        FzR = self.mR0 * g # Vertical load at rear axle [N]

        # State variables
        X = states[0]              # X position [m]
        Y = states[1]              # Y position [m]
        PSI = states[2]            # Yaw angle [rad]
        v = states[3]              # Velocity [m/s]
        ALPHAT = states[4]         # Slip angle [rad]
        dPSI = states[5]           # Yaw rate [rad/s]

        # Get controls
        deltaf = self.steering.get_steering_angle(t,states) # Current steering angle [rad]
        FxF = self.engine_f.get_long_force(t,states)        # Current longitudinal force at front axle [N]
        FxR = self.engine_r.get_long_force(t,states)        # Current longitudinal force at rear axle [N]

        # Slip angles
        ALPHAF = np.arctan2((v * np.sin(ALPHAT) + a * dPSI), (v * np.cos(ALPHAT))) - deltaf
        ALPHAR = np.arctan2((v * np.sin(ALPHAT) - b * dPSI), (v * np.cos(ALPHAT)))

        # Characteristic curve
        FyF = nF * self.tire.characteristic(ALPHAF, FzF/nF, muy)
        FyR = nR * self.tire.characteristic(ALPHAR, FzR/nR, muy)

        # Equations of motion
        dx = np.zeros(6)
        dx[0] = v * np.cos(ALPHAT + PSI)
        dx[1] = v * np.sin(ALPHAT + PSI)
        dx[2] = dPSI
        dx[3] = (FxF * np.cos(ALPHAT - deltaf) + FxR * np.cos(ALPHAT) + FyF * np.sin(ALPHAT - deltaf) + FyR * np.sin(ALPHAT))/(m)
        dx[4] = ( - FxF * np.sin(ALPHAT - deltaf) - FxR * np.sin(ALPHAT) + FyF * np.cos(ALPHAT - deltaf) + FyR * np.cos(ALPHAT) - m * v * dPSI) / (m * v)
        dx[5] = (FxF * a * np.sin(deltaf) + FyF * a * np.cos(deltaf) - FyR * b) / I
        
        return dx