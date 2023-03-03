#!/usr/bin/env python3
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread, pyqtSignal
import pyqtgraph as pg

class Plotter(QWidget):
    def __init__(self,plot_settings,background_color=(250,250,250)):
        super().__init__()
        pg.setConfigOption('background', background_color)
        pg.setConfigOptions(antialias=True)
        
        self.plot = pg.GraphicsLayoutWidget()
        self.plots = []
        self.plot_settings = plot_settings
        
        # Setting up different plots using plot settings
        for plot in plot_settings:
            new_plot = self.plot.addPlot(row=plot['row'],col=plot['col'],colspan=plot['colspan'],title=plot['title'])
            new_plot.setLabel('left',plot['ylabel'])
            new_plot.setLabel('bottom',plot['xlabel'])
            new_plot.setAspectLocked(lock=True,ratio=1)
            new_plot.setAutoPan(5,5)
            new_plot.setYRange(-3, 3)
            self.plots.append(new_plot)
            
        self.widget_layout = QVBoxLayout()
        self.widget_layout.addWidget(self.plot)
        self.setLayout(self.widget_layout)      
            
    def plot_data(self, data):
        for idx,plot in enumerate(self.plots):
            plot.clear()
            plot.plot(data[idx][0], data[idx][1], pen=pg.mkPen(color=data[idx][2], width=2))
