"""
This demo demonstrates how to draw a dynamic matplotlib 
plot in a wxPython application.

This code is based on Eli Bendersky's code found here:
http://eli.thegreenplace.net/files/prog_code/wx_mpl_dynamic_graph.py.txt

"""

import os
import random
import wx
import serial
import threading
import json
import time

import matplotlib
matplotlib.use('WXAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg
import numpy as np
import pylab
import math


class DataSource(object):

    read_data = True

    s = serial.Serial(port = '/dev/ttyACM0', baudrate = 115200) #SensorTag
    # start read_loop in a separate thread
    
    


    def __init__(self, graph):
        self.graph = graph

        # start separate thread to generate dummy data
        t1 = threading.Thread(target=self.read_loop, args=())
        t1.start()

    def close(self):
      self.read_data = False

    # read loop for dummy data


    def read_loop(self):
        output = ''
        x = 1;
        sensor_data = {"":""};
        sensor_data.clear
        while self.read_data:
            try:
                data = self.s.read();
                if len(data) > 0:

                    if (data[-1] !='\n'):
                        output += data
                    elif (data[-1]=='\n'): 
                        print output;
                        sensor_data =  json.loads(json.dumps(output));                  
                        output = ''
                        print type(sensor_data)
                        if (sensor_data["id"] == "temp_007"):
                            
                            if isinstance(self.graph, wx.Frame):
                                print "123 ", sensor_data["value"]["value"]
                                self.graph.update_data(int(sensor_data["value"]["value"]), 0) 
                                wx.CallAfter(self.graph.draw_plot)

                    
                        """ elif  (sensor_data["id"] == "bmp_280"):

                            if isinstance(self.graph, wx.Frame):

                                if (sensor_data["value"]["id"] == "pressure"):                                    
                                    self.graph.update_data(int(sensor_data["value"]["value"]),1) 

                                elif (sensor_data["value"]["id"] == "attitude"):
                                    self.graph.update_data(int(sensor_data["value"]["value"]),2) 

                            wx.CallAfter(self.graph.draw_plot)
                        # update plot"""

                        sensor_data.clear;
                        


            except Exception, e:
                print "Exception:", e, "12345"

        # close serial port
        print "close serial port"
        self.s.close()




class GraphFrame(wx.Frame):
    """ The main frame of the application
    """
    title = 'Demo'
    
    def __init__(self):
        wx.Frame.__init__(self, None, -1, self.title)
        
    # handle window close event    
        self.Bind(wx.EVT_CLOSE, self.on_exit)

        # set data source
        self.source = DataSource(self)
        
        self.data1 = []
        self.data2 = []
        self.data3 = []
        
        self.create_main_panel()
        

    def create_main_panel(self):

        self.panel = wx.Panel(self)

        self.init_plot()
        self.canvas = FigureCanvasWxAgg(self.panel, -1, self.fig)

        self.vbox = wx.BoxSizer(wx.VERTICAL)
        self.vbox.Add(self.canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)        
        
        self.panel.SetSizer(self.vbox)
        self.vbox.Fit(self)
   

    def init_plot(self):
        self.fig = Figure((6.0, 3.0), dpi=100)

        self.axes = self.fig.add_subplot(111)
        self.axes.set_axis_bgcolor('black')
        self.axes.set_title('Sensor data', size=12)

        self.axes2 = self.axes.twinx();
        self.axes3 = self.axes.twinx();
        
        pylab.setp(self.axes.get_xticklabels(), fontsize=8)
        pylab.setp(self.axes.get_yticklabels(), fontsize=8)

        # plot the data as a line series, and save the reference 
        # to the plotted line series
        self.plot_data1 = self.axes.plot(self.data1, linewidth=1, color=(1, 0, 0))[0]
        self.plot_data2 = self.axes.plot(self.data2, linewidth=1, color=(0, 1, 0))[0]
        self.plot_data3 = self.axes.plot(self.data3, linewidth=1, color=(0, 0, 1))[0]

    def update_data(self, sensor1, p):
        if (p == 0):
            self.data1.append(sensor1)

        if (p == 1):
            self.data2.append(sensor2)

        if (p == 0):
            self.data3.append(sensor3)


    def draw_plot(self):
        """ Redraws the plot
        """

        xmax = len(self.data1) if len(self.data1) > 50 else 50
        xmin = xmax - 50

        ymin = round(min(self.data1), 0) - 1
        ymax = round(max(self.data1), 0) + 1
       
        self.axes.set_xbound(lower=xmin, upper=xmax)
        self.axes.set_ybound(lower=ymin, upper=ymax)
        
        self.axes.grid(True, color='gray')
        pylab.setp(self.axes.get_xticklabels(), visible=True)
       
        self.plot_data1.set_xdata(np.arange(len(self.data1)))
        self.plot_data1.set_ydata(np.array(self.data1))

        self.plot_data2.set_xdata(np.arange(len(self.data2)))
        self.plot_data2.set_ydata(np.array(self.data2))

        self.plot_data3.set_xdata(np.arange(len(self.data3)))
        self.plot_data3.set_ydata(np.array(self.data3))
        
        self.canvas.draw()

    
    def on_exit(self, event):
        self.source.close()
        self.Destroy()
    
    

if __name__ == '__main__':
    app = wx.PySimpleApp()
    app.frame = GraphFrame()
    app.frame.Show()
    app.MainLoop()

