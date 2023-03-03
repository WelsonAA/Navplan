#!/usr/bin/env python3
import numpy as np
import pygame
import os
from .sprites import *
import time
from .global_vars import *




class Environment:
    '''
    Environment that wraps around the simulator to provide a graphical environment along with graphing capabilities
    Parameters:
        simulator (Simulator object): simulator object with vehicle model initialized. Used to run physics in the environment
        dt (float [sec]): step_size for each step done in the environment, also used to determine the FPS of the rendering
        api_control (bool): if True, only the vehicle api is allowed to control the car. False allows control using keyboard (and also the 
                            vehicle api but is not recommended to use 2 different modes at the same time)
        graphics (bool): enable graphics or not, could be disable and only view graphs of the car position, velocity, etc.
    '''
    def __init__(self,dt,api_control=False,theme='light'):
        self.dt = dt
        self.manual_control_steering_angle = 15
        self.graphics = True
        self.car_states = None
        self.current_state = [0,0,0,0,0,0]
        self.num_updates = 0
        
        if theme == 'light':
           self.theme = LightTheme
        elif theme == 'dark':
            self.theme = DarkTheme
        else:
            print("Error, theme: {} is undefined".format(theme))
            
        pygame.init()
        #pygame.mixer.init()
        self.screen_size = screen_size
        self.FPS = int(1/self.dt)

        self.screen = pygame.display.set_mode((self.screen_size[0],self.screen_size[1]))
        pygame.display.set_caption("NavPlan Car Simulator")
        self.clock = pygame.time.Clock()

        self.all_sprites = pygame.sprite.Group()
        self.vehicle_sprites = pygame.sprite.Group()
        self.car_sprite = CarSprite(4,1.5)
        self.background = GridBackground(self.screen_size,theme=self.theme)
        self.paths = []
        self.path_params = [] # Used to publish back to ros
        self.cone_params = [] # Used to publish back to ros
        self.cones = []
        self.all_sprites.add(self.background)
        self.vehicle_sprites.add(self.car_sprite)
        self.camera_pos = np.zeros(2) # in meters
        self.follow = False
        self.PPM = 10 # Pixels per meter, basically used to describe the height of the camera

    def add_cone(self,x,y,radius=1,style=0):
        new_cone = Cone([x,y],self.screen_size,radius=radius,style=style)
        self.cone_params.append([x,y,radius,style])
        self.cones.append(new_cone)
        self.all_sprites.add(new_cone)
            
    def add_path(self,points,spline_degree=3,smoothness=0,resolution=100, color=None, width=3):
        '''
        '''
        points = np.array(points)
        path_color = self.theme['path_color'] if color is None else color
        new_path = Path(points,self.screen_size,
                        smoothness=smoothness,
                        resolution=resolution,
                        spline_degree=spline_degree,
                        color=path_color,
                        width=width)
        path_desc = [7+points.shape[0]*2,spline_degree,smoothness,resolution,*path_color,width,*points.reshape(-1)] # Used for ros
        self.path_params.append(path_desc)
        self.paths.append(new_path)
        self.all_sprites.add(new_path)
        
    def get_dist_to_path(self,path_index,only_waypoints=False):
        '''
        '''
        path = self.paths[path_index].smooth_points
        
        if only_waypoints:
            path = self.paths[path_index].pos_arr
            
        dists = path - np.array(self.current_state[:2]).reshape(1,2)
        min_dist = np.min(np.linalg.norm(dists,axis=1))
        return min_dist
    
    def update_state(self,states):
        self.current_state = states
        
    def step(self):
        '''
        Step the environment's simulation for one step this includes:
            - Checking for keyboard input and using that if needed
            - Stepping the physics simulation using the vehicle model and simulator
            - Updating the rendered graphics if needed
        '''
        self.handle_manual_inputs()
        
        # Step physics simulator and save the states
        #self.simulator.step(step_size=self.dt) Disabled in ros based version
        
        if self.car_states is None:
            self.car_states = np.array(self.current_state).reshape(1,-1)
        else:
            self.car_states = np.vstack((self.car_states,np.array(self.current_state).reshape(1,-1)))
        self.num_updates += 1

        # Update graphics if needed
        if self.graphics:
            car_state = self.current_state[:3] # [car_x_pos, car_y_pos, car_rot]

            if self.follow: # Set camera position to the same as the car if following it
                self.camera_pos[0] = car_state[0]
                self.camera_pos[1] = car_state[1]*-1

            car_pos_pixel = self.world_to_pixel(*car_state[:2]) # Car position in pixel space

            # Update car, background, and path sprites
            self.car_sprite.update(*car_pos_pixel,car_state[2],self.PPM)
            self.background.update(self.pixel_to_world,self.world_to_pixel)
            for idx,path in enumerate(self.paths):
                path.update(self.world_to_pixel)
                path.path_dists.append(self.get_dist_to_path(idx))
                
            for idx,cone in enumerate(self.cones):
                cone.update(self.world_to_pixel,self.PPM)

    def handle_manual_inputs(self): # Used to handle scrolling through the map or zooming in and out
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if not self.follow and event.button == 1:
                    self.mouse_down = True
                    self.prev_mouse_pos = event.pos

                if event.button == 4:
                    self.PPM += 1
                if event.button == 5:
                    self.PPM -= 1
                    self.PPM = np.max([self.PPM,1])

            elif not self.follow and event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.mouse_down = False

            elif  not self.follow and event.type == pygame.MOUSEMOTION:
                if self.mouse_down:
                    mouse_x, mouse_y = event.pos
                    self.camera_pos[0] -= (mouse_x - self.prev_mouse_pos[0])/(self.PPM)
                    self.camera_pos[1] -= (mouse_y - self.prev_mouse_pos[1])/(self.PPM)
                    self.prev_mouse_pos = event.pos
        
    def render(self,graphs=False):
        '''
        Show the current graphics onto the pygame screen and draw graphs if needed
        '''
        self.screen.fill((255,255,255))
        if not graphs:
            self.all_sprites.draw(self.screen)
            self.vehicle_sprites.draw(self.screen)
            pygame.display.flip()
    
    def world_to_pixel(self,x,y):
        '''
        Transforms a point (x,y) given in absolute world coordinates in meters
        to a point (x,y) that represents a pixel where x is the horizontal location of the pixel on the screen
                                                   and y is the vertical location of the pixel on the screen
        Note that pygame pixel coordinates have the (0,0) in the top left corner
        '''
        point = np.array([x,-1*y]) # -1 because the top left corner in pygame is zero and increases as you go down, so flip the y axis
        return np.floor( (point-self.camera_pos)*self.PPM + self.screen_size/2 )
    
    def pixel_to_world(self,x,y):
        '''
        Transforms a point (x,y) given in pixel coordinates (pygame) to absolute world coordinates in meters
        Note that pygame pixel coordinates have the (0,0) in the top left corner
        '''
        point = np.array([x,y])
        point = (point-self.screen_size/2)/self.PPM + self.camera_pos
        point[1] *= -1   # -1 because the top left corner in pygame is zero and increases as you go down, so flip the y axis
        return point
    
    def play(self,max_frames=None,graphs=False,funcs_call=None):
        num_frames = 0
        self.running = True
        self.mouse_down = False
        self.prev_mouse_pos = [0,0]
        while self.running and (max_frames is None or num_frames<max_frames):
            num_frames += 1
            self.clock.tick(self.FPS)
            self.step()
            self.render(graphs=graphs)
            if funcs_call is not None:
                for f in funcs_call:
                    f()

        pygame.quit()
