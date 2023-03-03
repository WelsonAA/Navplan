#!/usr/bin/env python3
import numpy as np
import pygame
import os
from .sprites import *
import matplotlib.pyplot as plt
import matplotlib.backends.backend_agg as agg
import pylab
import matplotlib
matplotlib.use("Agg")
import time
from scipy import interpolate
from .global_vars import *

class CarSprite(pygame.sprite.Sprite):
    '''
    CarSprite contains the graphics for the car, it is responsible for updating the car sprite and drawing it when .draw() is called
    Parameters:
        car_length (float): Length of the car [m]
        car_width (float): Width of the car [m]
    '''
    def __init__(self,car_length,car_width):
        pygame.sprite.Sprite.__init__(self)
        self.car_length = car_length
        self.car_width = car_width
        
        # Car image used, can be changed by changing the "car.png" below
        game_folder = os.path.dirname(__file__)
        self.car_img = pygame.image.load(os.path.join(game_folder,"car.png")).convert()
        
        self.image = None
        self.rect = None
        self.mask = None
        
    def update(self,x,y,angle,PPM):
        '''
        Update location of the car sprite in pixel space
        Args:
            x (float): x location of the center of the car in pixel space
            y (float): y location of the center of the car in pixel space
            angle (float): current yaw angle of the car [rad]
            PPM (int): pixels per meter is the number of pixels a meter occupies, used to determine the sprite size
        '''
        pixel_car_length = int(self.car_length*PPM)
        pixel_car_width = int(self.car_width*PPM)
        
        # Scale and rotate the car img
        self.image = pygame.transform.rotate(pygame.transform.scale(self.car_img,(pixel_car_length,pixel_car_width)), angle*180/np.pi)
        self.image.set_colorkey((0,0,0))
        
        self.rect = self.image.get_rect()
        self.mask = pygame.mask.from_surface(self.image)
        
        # Position car img in pixel space using its center
        self.rect.centerx = x
        self.rect.centery = y

        
class GridBackground(pygame.sprite.Sprite):
    '''
    GridBackground draw the x and y axes, a fine and coarse grid along with numbers on the axes.
    It takes into account the position of the camera along with how zoom in it is in order to provide correct coordinates
    Parameters:
        screen_size: screen_size[0] is the screen width, screen_size[1] is the screen height, both measured in number of pixels
        theme (dict): theme dictionary that contains background theme, see themes in environment.py to see what the dict contains
    '''
    def __init__(self,screen_size,theme):
        pygame.sprite.Sprite.__init__(self)
        self.screen_size = screen_size
        self.theme = theme
        
        self.image = None
        self.rect = None
            
        self.fonts = []
        for i in range(len(self.theme['font'])):
            self.fonts.append(pygame.font.SysFont(self.theme['font'][i], self.theme['font_sizes'][i]))
            
    def update(self,pixel_to_world,world_to_pixel):
        '''
        Update current grid image according to the camera position and how zoom in the view is
        Args:
            pixel_to_world (function takes x,y as input): transforms any point from pixel coordinates to world coordinates
            world_to_pixel (function takes x,y as input): transforms any point from world coordinates to pixel coordinates 
        '''
        w = self.screen_size[0]   # Screen width [num of pixels]
        h = self.screen_size[1]   # Screen height [num of pixels]
        m_w = self.theme['margins'][0]     # Screen width/x-axis margin [num of pixels]
        m_h = self.theme['margins'][1]     # Screen height/y-axis margin [num of pixels]
        
        # Create grid image
        image = pygame.Surface((w,h))
        image.fill(self.theme['back_color']) # White background
        
        # Draw x and y axes
        pygame.draw.line(image, self.theme['axes_color'], (m_w, h-m_h), (w, h-m_h))  # X axis
        pygame.draw.line(image, self.theme['axes_color'], (m_w, h-m_h), (m_w, 0))    # Y axis
        
        # Determine x and y values of axes intersection
        start_pos = pixel_to_world(0,0)
        
        # Draw different grid lines
        for i in range(len(self.theme['grid_sizes'])):
            image = self.draw_grid_lines(image,*start_pos,i,world_to_pixel)
        
        self.image = image
        self.rect = self.image.get_rect()
        
    def draw_grid_lines(self,image,start_x,start_y,i,world_to_pixel):
        '''
        Draw the squares or grid lines on the background
        Args:
            image (pygame.Surface): surface to draw the grid lines on
            start_x (float): the x coordinate in world coordinates at (0,0) in pixel coordinates which is in the top left corner
            start_y (float): the y coordinate in world coordinates at (0,0) in pixel coordinates which is in the top left corner
            i (int): the integer representing which level of grids are being drawn
            world_to_pixel (function takes x,y as input): transforms any point from world coordinates to pixel coordinates 
        Returns:
            image (pygame.Surface): the image with draw grid lines on it
        '''
        w = self.screen_size[0]   # Screen width [num of pixels]
        h = self.screen_size[1]   # Screen height [num of pixels]
        m_w = self.theme['margins'][0]     # Screen width/x-axis margin [num of pixels]
        m_h = self.theme['margins'][1]     # Screen height/y-axis margin [num of pixels]
        
        grid_size = self.theme['grid_sizes'][i]
        color = self.theme['grid_colors'][i]
        font = self.fonts[i]
        
        # Draw vertical grid lines
        current_x = (start_x//grid_size[0])*grid_size[0]
        current_x_pixel = world_to_pixel(current_x,0)[0]
        while current_x_pixel < w:
            
            # Condition to not overwrite grid lines from lower levels with higher levels
            grid_sizes_bool = True
            for j in range(0,i):
                if current_x%self.theme['grid_sizes'][j][0] == 0:
                    grid_sizes_bool = False
                    break
                    
            if current_x_pixel > m_w and grid_sizes_bool:
                # Draw line
                pygame.draw.line(image,color,(current_x_pixel, h-m_h), (current_x_pixel, 0))
                
                # Draw number as text
                textsurface = font.render(str(int(current_x)), False, self.theme['font_color'])
                image.blit(textsurface,(current_x_pixel,h-(m_h)))
                
            # Move to the next line
            current_x += grid_size[0]
            current_x_pixel = world_to_pixel(current_x,0)[0]
          
        # Draw horizontal grid lines
        current_y = (start_y//grid_size[1]+2)*grid_size[1]
        current_y_pixel = world_to_pixel(0,current_y)[1]
        while current_y_pixel < h:
            
            # Condition to not overwrite grid lines from lower levels with higher levels
            grid_sizes_bool = True
            for j in range(0,i):
                if current_y%self.theme['grid_sizes'][j][1] == 0:
                    grid_sizes_bool = False
                    break
            if current_y_pixel < h-m_h and grid_sizes_bool:
                # Draw line
                pygame.draw.line(image,color,(m_w,current_y_pixel), (w,current_y_pixel))
                
                # Draw number as text
                textsurface = font.render(str(int(current_y)), False, self.theme['font_color'])
                num_chars = len(str(int(current_y))) # Used to adjust position of the number according to how many characters there are
                image.blit(textsurface,(m_w-self.theme['font_sizes'][i]/1.5*num_chars,current_y_pixel))
                
            # Move to the next line, note here the lines are moving from top to bottom in world coordinates
            current_y -= grid_size[1]
            current_y_pixel = world_to_pixel(0,current_y)[1]
            
        return image
        
class Path(pygame.sprite.Sprite):
    '''
    Path class responsible for interpolating waypoints and drawing a curve on the screen
    Parameters:
        pos_arr (np.array or list, shape=(N,2)): N = number of waypoints, to close path, need the last point to be the same as the first
        screen_size: screen_size[0] is the screen width, screen_size[1] is the screen height, both measured in number of pixels
        smoothness (float): the higher the more the curve will deviate from the waypoints to be smoother, 0 will follow waypoints exactly
        spline_degree (int): degree of the polynomial fitted on the spline, 1 is linear, 2 is quadratic, etc.
        resolution (int): number of points to use to draw the curve (the curve will be splitted into this number of points).
            Note this will also affect the estimate of the car's distance to the path, resolution of the same number as the waypoints means
            the closest point to the car will be one of the waypoints
        color (tuple len=3): the color of the path
        width (int): width to draw the path with
    '''
    def __init__(self,pos_arr,screen_size,smoothness=0,spline_degree=3,resolution=100,color=(0,0,0),width=3):
        pygame.sprite.Sprite.__init__(self)
        self.pos_arr = np.array(pos_arr)
        self.screen_size = screen_size
        self.resolution = int(resolution)
        self.color = color
        self.width = width
        self.path_dists = []
        
        # Calculating smooth curve using polynomial interpolation
        self.tck, self.u = interpolate.splprep(self.pos_arr.T, s=smoothness, k=int(spline_degree))
        unew = np.linspace(0,1,self.resolution)
        
        # Array of points in world coordinates each representing a point on the path, shape=(self.resolution,2)
        self.smooth_points = np.array(interpolate.splev(unew, self.tck)).T
        
        self.image = None
        self.rect = None
    
    def update(self,world_to_pixel):
        '''
        Update the current sprite of the path
        Args:
            world_to_pixel (function takes x,y as input): transforms any point from world coordinates to pixel coordinates 
        '''
        w = self.screen_size[0]   # Screen width [num of pixels]
        h = self.screen_size[1]   # Screen height [num of pixels]
        
        # Transforming the path's points in world coordinates to pixel coordinates
        pos_arr_pixel = []
        for point in self.smooth_points:
            point_pixel = world_to_pixel(*point)
            pos_arr_pixel.append(point_pixel)
        
        # Creating transparent background for the path
        image = pygame.Surface((w,h))
        back_fill = (0,0,0)
        if back_fill == self.color:
            back_fill = (255,255,255)
        image.fill(back_fill)
        image.set_colorkey(back_fill) # Completely transparent background
        
        # Draw line between every 2 points on the path (doesn't connect the first point with the last point)
        for i in range(len(pos_arr_pixel)-1):
            pygame.draw.line(image,self.color,pos_arr_pixel[i],pos_arr_pixel[i+1],width=self.width)
        
        # Draw each waypoint as an X symbol
        pos_arr_pixel = []
        for point in self.pos_arr:
            point_pixel = world_to_pixel(*point)
            image = self.draw_x(*point_pixel,image,color=self.color)
        
        # Save the new image as the path's image
        self.image = image
        self.rect = self.image.get_rect()
        
    def draw_x(self,x,y,image,color):
        '''
        Draw the symbol X at a given point in pixel coordinates
        Args:
            x (int): x pixel position of the X
            y (int): y pixel position of the X, note that y=0 is at the top not the bottom
            image (pygame.Surface): surface to draw the X on
            color (tuple len=3): color of the X to draw
        '''
        x_size = 7 # Default value size for the X
        pygame.draw.line(image,color,(x-x_size,y-x_size),(x+x_size,y+x_size),width=2*self.width)
        pygame.draw.line(image,color,(x-x_size,y+x_size),(x+x_size,y-x_size),width=2*self.width)
        return image
    


class Cone(pygame.sprite.Sprite):
    '''
    Cone class responsible for drawing 1 cone on the screen and sampling its position with noise
    Parameters:
        world_pos (np.array or list, shape=(,2)): The position of the cone in world coordinates
        variance: 
        screen_size: screen_size[0] is the screen width, screen_size[1] is the screen height, both measured in number of pixels
        radius (float): Used to increase the size of the cone by that amount (only for drawing purposes)
        style (arr): See Yellow_Cone above to understand what the components of this is. Its main purpose is to know the color and shape of
            of the cone
    '''
    def __init__(self,world_pos,screen_size,variance=1,radius=1,style=0):
        pygame.sprite.Sprite.__init__(self)
        self.world_pos = world_pos
        self.screen_size = screen_size
        self.radius = radius
        if style==0:
            self.style = Yellow_Cone
        elif style == 1:
            self.style = Blue_Cone
        else:
            self.style = Red_Cone
        self.variance = variance
        
        self.image = None
        self.rect = None
        
    def update(self,world_to_pixel,PPM):
        '''
        Update the current sprite of the cone
        Args:
            world_to_pixel (function takes x,y as input): transforms any point from world coordinates to pixel coordinates 
        '''
        w = self.screen_size[0]   # Screen width [num of pixels]
        h = self.screen_size[1]   # Screen height [num of pixels]
        
        # Transforming the path's points in world coordinates to pixel coordinates
        pixel_pos = world_to_pixel(*self.world_pos)
        
        # Creating transparent background for the path
        image = pygame.Surface((w,h))
        back_fill = (0,0,0)
        image.fill(back_fill)
        image.set_colorkey(back_fill) # Completely transparent background
        
        # Draw each cone
        for s in self.style:
            pygame.draw.circle(image,s['color'],pixel_pos,s['radius']*PPM*self.radius)
        
        # Save the new image as the path's image
        self.image = image
        self.rect = self.image.get_rect()
        
    def sample(self):
        '''
        Sample the world position of the cone with noise generated using functions given in the variance array
        
        Returns:
            np.array (dtype=float): shape=(,2), first element is x world position, and second element is y world position
        '''
        return np.array([self.world_pos[0]+self.variance[0].sample(),self.world_pos[1]+self.variance[1].sample()],dtype=np.float)
