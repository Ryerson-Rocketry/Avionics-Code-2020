import tkinter as tk
from tkinter import ttk
from tkinter import *
from PIL import Image
from PIL import ImageTk
from itertools import count
import pandas as pd

#from cartopy import config
#import cartopy.crs as ccrs

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation
import time, sys, struct, copy, os#, serial
#import decoder_rrc_v2_2
#ser = serial.Serial('/dev/ttyACM0', 9600) # change for actual devices
#decoder_rrc_v2_2()
#Screensize
window = Tk()
window.resizable(0, 0) # this prevents from resizing the window
window.title("Ground Station GUI")
window.configure(background='LightSkyBlue1')
screenX = 800
screenY = 500

#Styles
COLOR1 = 'LightSkyBlue1'
s = ttk.Style()
s.configure('RRC.Label', background=COLOR1, anchor="center")
s.configure('RRC.TPanedwindow', background=COLOR1)
s.configure('RRC.TNotebook', background=COLOR1)

# Import the Notebook.tab element from the default theme
s.element_create('Plain.Notebook.tab', "from", 'default')
# Redefine the TNotebook Tab layout to use the new element
s.layout("TNotebook.Tab",
    [('Plain.Notebook.tab', {'children':
        [('Notebook.padding', {'side': 'top', 'children':
            [('Notebook.focus', {'side': 'top', 'children':
                [('Notebook.label', {'side': 'top', 'sticky': ''})],
            'sticky': 'nswe'})],
        'sticky': 'nswe'})],
    'sticky': 'nswe'})])
s.configure("TNotebook.Tab", background=COLOR1)

#organization = toolbar[options], window[graph[data notebook], map, logo],statusbar
window.panes = tk.ttk.PanedWindow(orient="vertical", width = screenX, height=screenY,style="RRC.TPanedwindow")
panes = window.panes
panes.pack(fill="both",expand=1)

#toolbar = tk.PanedWindow(panes, orient="horizontal")
#options = tk.Button(toolbar, text="options")
#toolbar.add(options)
#options2 = tk.Button(toolbar, text="options")
#toolbar.add(options2)
#options3 = tk.Button(toolbar, text="options")
#toolbar.add(options3)
#optionsEmpty = tk.Label(toolbar) #Without this the buttons will stretch to fill the space
#toolbar.add(optionsEmpty)
#panes.add(toolbar,minsize=25)
    

interface = tk.ttk.PanedWindow(panes, orient="horizontal", height=475, style="RRC.TPanedwindow")

data = tk.ttk.Notebook(interface, style="RRC.TNotebook", width=400)
overview = tk.ttk.PanedWindow(data, orient="horizontal", style="RRC.TPanedwindow")
overview1 = tk.ttk.PanedWindow(overview, orient="vertical", width=200,style="RRC.TPanedwindow")
latitudeLabel = ttk.Label(overview1, text="Latitude",style="RRC.Label")
overview1.add(latitudeLabel)
latitude = ttk.Label(overview1, text="0.0",style="RRC.Label")
overview1.add(latitude)
longitudeLabel = ttk.Label(overview1, text="Longitude",style="RRC.Label")
overview1.add(longitudeLabel)
longitude = ttk.Label(overview1, text="0.0",style="RRC.Label")
overview1.add(longitude)
temperatureLabel = ttk.Label(overview1, text="Temperature",style="RRC.Label")
overview1.add(temperatureLabel)
temperature = ttk.Label(overview1, text="0",style="RRC.Label")
overview1.add(temperature)
pressureLabel = ttk.Label(overview1, text="Pressure",style="RRC.Label")
overview1.add(pressureLabel)
pressure = ttk.Label(overview1, text="0.0",style="RRC.Label")
overview1.add(pressure)
flighttimeLabel = ttk.Label(overview1, text="Flight Time",style="RRC.Label")
overview1.add(flighttimeLabel)
flighttime = ttk.Label(overview1, text="0",style="RRC.Label")
overview1.add(flighttime)
spacer1 = ttk.Label(overview1, text="",style="RRC.Label")
overview1.add(spacer1)
overview.add(overview1)

overview2 = tk.ttk.PanedWindow(overview, orient="vertical",style="RRC.TPanedwindow")
speedLabel = ttk.Label(overview2, text="Speed",style="RRC.Label")
overview2.add(speedLabel)
speed = ttk.Label(overview2, text="0",style="RRC.Label")
overview2.add(speed)
altitudeLabel = ttk.Label(overview2, text="Altitude",style="RRC.Label")
overview2.add(altitudeLabel)
altitude = ttk.Label(overview2, text="0",style="RRC.Label")
overview2.add(altitude)
accelxLabel = ttk.Label(overview2, text="AccelX",style="RRC.Label")
overview2.add(accelxLabel)
accelx = ttk.Label(overview2, text="0",style="RRC.Label")
overview2.add(accelx)
accelyLabel = ttk.Label(overview2, text="AccelY",style="RRC.Label")
overview2.add(accelyLabel)
accely = ttk.Label(overview2, text="0",style="RRC.Label")
overview2.add(accely)
accelzLabel = ttk.Label(overview2, text="AccelZ",style="RRC.Label")
overview2.add(accelzLabel)
accelz = ttk.Label(overview2, text="0",style="RRC.Label")
overview2.add(accelz)
spacer2 = ttk.Label(overview2, text="",style="RRC.Label")
overview2.add(spacer2)
overview.add(overview2)

data.add(overview, text="Overview")


#Atmospheric Data

x = [0]
y = [0]
index = count()

atmos = tk.PanedWindow(data, orient="vertical")
pressureGraph = tk.Frame(atmos)
plt.ion()
pressureFigure = plt.figure(figsize=(3,3), dpi=100)
pressurePlot = pressureFigure.add_subplot(111)
pressurePlot.plot(x,y)

pressureCanvas = FigureCanvasTkAgg(pressureFigure, pressureGraph)
pressureCanvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

atmos.add(pressureGraph)
data.add(atmos, text="Pressure")

atmos2 = tk.PanedWindow(data, orient="vertical")
temperatureGraph = tk.Frame(atmos2)
plt.ion()
temperatureFigure = plt.figure(figsize=(3,3), dpi=100)
temperaturePlot = temperatureFigure.add_subplot(111)
temperaturePlot.plot(x,y)  

temperatureCanvas = FigureCanvasTkAgg(temperatureFigure, temperatureGraph)
temperatureCanvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

atmos2.add(temperatureGraph)
data.add(atmos2, text="Temperature")
    
#Flight data
flightdata = tk.PanedWindow(data, orient="vertical")
altitudeGraph = tk.Frame(flightdata)
plt.ion()
altitudeFigure = plt.figure(figsize=(3,3), dpi=100)
altitudePlot = altitudeFigure.add_subplot(111)
altitudePlot.plot(x,y)  

altitudeCanvas = FigureCanvasTkAgg(altitudeFigure, altitudeGraph)
altitudeCanvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

flightdata.add(altitudeGraph)
data.add(flightdata, text="Altitude")


flightdata2 = tk.PanedWindow(data, orient="vertical")
speedGraph = tk.Frame(flightdata2)
plt.ion()
speedFigure = plt.figure(figsize=(3,3), dpi=100)
speedPlot = speedFigure.add_subplot(111)
speedPlot.plot(x,y)  

speedCanvas = FigureCanvasTkAgg(speedFigure, speedGraph)
speedCanvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

flightdata2.add(speedGraph)
data.add(flightdata2, text="Speed")
    
#Acceleration
accel = tk.PanedWindow(data, orient="vertical")
accelGraph = tk.Frame(accel)
plt.ion()
accelFigure = plt.figure(figsize=(3,3), dpi=100)
accelPlot = accelFigure.add_subplot(111)
accelPlot.plot(x,y)  

accelCanvas = FigureCanvasTkAgg(accelFigure, accelGraph)
accelCanvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

accel.add(accelGraph)
data.add(accel, text="Acceleration")


interface.add(data)

rightInterface = tk.ttk.PanedWindow(interface, orient="vertical", width=400, style="RRC.TPanedwindow")
gpsBackground = tk.Frame(rightInterface, height=300)

gps = plt.figure(figsize=(3, 4))

fname = "Images/Large_Map.png"
img_extent = (-106.967445, -106.858726, 32.925260, 32.979024)
img = plt.imread(fname)

'''
ax = plt.axes(projection=ccrs.PlateCarree())

ax.imshow(img, origin='upper', extent=img_extent, transform=ccrs.PlateCarree())
'''
gpsCanvas = FigureCanvasTkAgg(gps, gpsBackground)
gpsCanvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
rightInterface.add(gpsBackground)
'''
ax.plot([-106.967445, -106.858726], [32.925260, 32.979024],
         color='blue', linewidth=2, marker='o',
         transform=ccrs.Geodetic(),
         )
'''

belowGPS = tk.ttk.PanedWindow(rightInterface, orient="horizontal",height=200, style="RRC.TPanedwindow")
belowGPSData = tk.ttk.PanedWindow(belowGPS, orient="vertical", width=290, style="RRC.TPanedwindow")
clock = ttk.Label(belowGPSData, text="current time",style="RRC.Label")
time_string = time.strftime('%H:%M:%S')
clock.config(text=time_string)
belowGPSData.add(clock)
altitudeLabel2 = ttk.Label(belowGPSData, text="Altitude",style="RRC.Label")
belowGPSData.add(altitudeLabel2)
altitude2 = ttk.Label(belowGPSData, text="0",style="RRC.Label")
belowGPSData.add(altitude2)
speedLabel2 = ttk.Label(belowGPSData, text="Speed",style="RRC.Label")
belowGPSData.add(speedLabel2)
speed2 = ttk.Label(belowGPSData, text="0",style="RRC.Label")
belowGPSData.add(speed2)
spacer3 = ttk.Label(belowGPSData, text="",style="RRC.Label")
belowGPSData.add(spacer3)

belowGPS.add(belowGPSData)

logo = tk.PhotoImage(file="Images/logo.png", name="logo", format="png") #loads image
logoElement = tk.Label(belowGPS, text="logo",image=logo,height=100, background=COLOR1) #actually adds the image        
logoElement.image = logo #for some reason, needs this to actually render
belowGPS.add(logoElement)

rightInterface.add(belowGPS)
interface.add(rightInterface)
    
panes.add(interface)


statusBar = tk.ttk.PanedWindow(panes, orient="horizontal", height=15, style="RRC.TPanedwindow")
errors = tk.ttk.Label(statusBar, text="errors", style="RRC.Label")
statusBar.add(errors)
indicators = tk.ttk.Label(statusBar, text="indicators", style="RRC.Label")
statusBar.add(indicators)
panes.add(statusBar)

def tick():
    time_string = time.strftime('%H:%M:%S')
    clock.config(text=time_string)



def dataUpdate(i):
    if (ser.in_waiting > 0):
        #print(ser.readline())
        x.append(next(index))
        y.append( ser.readline())
        #print(y[nextX])
        #print(nextX)
    if nextX%100 == 0: #Take this out if updates slow down.
        pressureFigure.canvas.draw()
        pressurePlot.cla()
        pressurePlot.plot(x,y)

def testUpdate(i):
    data = pd.read_csv('data.csv')
    x = data['x_value']
    y = data['total_1']
    latitude = data['Latitude']
    longitude = data['Longitude']
    pressurePlot.cla()
    pressurePlot.plot(x,y)
    #ax.plot(latitude,longitude) #Change so it doesn't stack lines, .cla() clears image as well so it doesn't work unless that is reloaded each time.

ani = FuncAnimation(plt.gcf(), testUpdate, interval=1000)

#while 1: # Replace with while in focus?
    #tick()
    #window.update()
    #dataUpdate(nextX) #Write backup every 5? minuntes
    #getSerial()
