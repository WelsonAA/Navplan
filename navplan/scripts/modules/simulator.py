#!/usr/bin/env python3
import numpy as np
from scipy import integrate

class Simulator:
    '''
    Simulator class is used to run the computation related to the simulation.
    It includes the vehicle model (and any other models) along with the time integrator
    Parameters needed to create an object:
        vehicle (VehicleSimple object): This is a vehicle model object which will have a tire model, engine model, and steering model
                                        also assumes that the parameters in vehicle model have been initialized
    '''
    def __init__(self,vehicle,initial_state=None,integrator="dopri5"):
        self.vehicle = vehicle                             # Vehicle Model
        self.num_state_vars = self.vehicle.num_state_vars  # Integer representing the number of variables in the state vector
        self.t = 0 # The time at te start of the simulation [sec]
        self.initial_state = np.zeros(self.num_state_vars) # Initial state sets all state variables to zero
        
        # Non-zero initial state
        if initial_state is not None:
            self.initial_state = initial_state
        
        self.current_state = np.copy(initial_state) # Current state defined as the initial state for the first time step
            
        self.integrator = integrator # String representing the type of time integrator to use
    
    def step_detail(self,step_size):
        '''
        
        '''
        end_t = self.t+step_size
        
        self.current_state = self.integrate(self.initial_state,0,end_t)
        self.t += step_size
        
    def step(self,step_size):
        '''
        
        '''
        end_t = self.t+step_size
        
        self.current_state = self.integrate(self.current_state,self.t,end_t)
        self.t += step_size
        
    def integrate(self,initial_state,initial_t,end_t):
        '''
        
        '''
        success = False
        val = 1
        for i in range(20):
            if np.abs(initial_state[3]) < (i+1) and self.integrator == 'dopri5':
                if self.vehicle.model(0,initial_state)[3] < 0:
                    initial_state[3] = -(i+1)
                else:
                    initial_state[3] = (i+1)

            integrator = integrate.ode(self.vehicle.model).set_integrator(self.integrator,beta=1e-1,nsteps=10e6)  # Set-up integrator
            integrator.set_initial_value(initial_state, initial_t)   # initial conditions of state and time

            y = integrator.integrate(end_t)
            if integrator.successful():
                break
        if not success:
            print("Couldn't integrate")

        return y
                 
    def full_simulate(self,tspan):
        '''
        
        '''
        t = np.linspace(*tspan)
        
        y = np.zeros((tspan[2], self.num_state_vars)) # array for solution
        y[0, :] = self.current_state
        
        r = integrate.ode(self.vehicle.model).set_integrator(self.integrator)  # choice of method
        r.set_initial_value(self.current_state, tspan[0])   # initial values
        
        for i in range(1, t.size):
            y[i, :] = r.integrate(t[i]) # get one more value, add it to the array
            if not r.successful():
                raise RuntimeError("Could not integrate")
        return y.T
        
