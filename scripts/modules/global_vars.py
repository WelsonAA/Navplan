#!/usr/bin/env python3
import numpy as np

screen_size = np.array([1200,700])
 
# Cone Colors    
Yellow_Cone = [ ### style = 0 ###
    {
        'color':(20,240,240),
        'radius':0.288
    },
    {
        'color':(10,10,10),
        'radius':0.144
    },
    {
        'color':(20,240,240),
        'radius':0.072
    }
]

    
Blue_Cone = [ ### style = 1 ###
    {
        'color':(20,20,240),
        'radius':0.288
    },
    {
        'color':(10,10,10),
        'radius':0.144
    },
    {
        'color':(20,20,240),
        'radius':0.072
    }
]

Red_Cone = [ ### style = anything but 0 or 1 ###
    {
        'color':(240,20,20),
        'radius':0.288
    },
    {
        'color':(10,10,10),
        'radius':0.144
    },
    {
        'color':(240,20,20),
        'radius':0.072
    }
]


# Themes to render grid backgroud and axes
                    
LightTheme = {
    'back_color' : (255,255,255),
    'axes_color' : (0,0,0),
    'grid_colors' : [(50,50,50),(150,150,150)],
    'grid_sizes':[(5,5),(1,1)],
    'margins':(35,100),
    'font':['Comic Sans MS','Comic Sans MS'],
    'font_sizes':[15,0],
    'font_color':(0,0,0),
    'path_color':(0,0,0)
}

DarkTheme = {
    'back_color' : (0,0,0),
    'axes_color' : (255,255,255),
    'grid_colors' : [(150,150,150),(50,50,50)],
    'grid_sizes':[(5,5),(1,1)],
    'margins':(35,100),
    'font':['Comic Sans MS','Comic Sans MS'],
    'font_sizes':[15,0],
    'font_color':(255,255,255),
    'path_color':(255,255,255)
}
