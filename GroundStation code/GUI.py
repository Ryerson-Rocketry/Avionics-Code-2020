from tkinter import ttk
from tkinter import *
import tkinter as tk
from PIL import Image
from PIL import ImageTk
from itertools import count
import pandas as pd

from cartopy import config
import cartopy.crs as ccrs

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation
import time, sys, struct, copy, os
'''
import decoder_rrc_v2_2 as decoder
while True:
    try:
        decoder.setPort('/dev/ttyUSB0', "57600")
        break
    except Exception as e:
        print(e)
print("Connected")
'''
#Screensize
window = Tk()
#window.resizable(0, 0) # this prevents from resizing the window
window.title("Ground Station GUI")
screenX = 800
screenY = 500

#organization = toolbar[options], window[graph[data notebook], map, logo],statusbar
window.panes = tk.PanedWindow(orient="vertical", width = screenX, height=screenY)
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
    

interface = tk.PanedWindow(panes, orient="horizontal")
interface.config(bg="blue")

data = tk.ttk.Notebook(interface)
overview = tk.PanedWindow(data, orient="horizontal")
overview1 = tk.PanedWindow(overview, orient="vertical")
latitudeLabel = tk.Label(overview1, text="Latitude")
overview1.add(latitudeLabel)
latitudeString = 0.0
latitude = tk.Label(overview1, text=latitudeString)
overview1.add(latitude)
longitudeLabel = tk.Label(overview1, text="Longitude")
overview1.add(longitudeLabel)
longitudeString = "0.0"
longitude = tk.Label(overview1, text=longitudeString)
overview1.add(longitude)
temperatureLabel = tk.Label(overview1, text="Temperature")
overview1.add(temperatureLabel)
temperature = tk.Label(overview1, text="0")
overview1.add(temperature)
pressureLabel = tk.Label(overview1, text="Pressure")
overview1.add(pressureLabel)
pressureString = "0.0"
pressure = tk.Label(overview1, text=pressureString)
overview1.add(pressure)
flighttimeLabel = tk.Label(overview1, text="Flight Time")
overview1.add(flighttimeLabel)
flighttime = tk.Label(overview1, text="0")
overview1.add(flighttime)
spacer1 = tk.Label(overview1, text="")
overview1.add(spacer1)
overview.add(overview1, minsize=200)

overview2 = tk.PanedWindow(overview, orient="vertical")
speedLabel = tk.Label(overview2, text="Speed")
overview2.add(speedLabel)
speed = tk.Label(overview2, text="0")
overview2.add(speed)
altitudeLabel = tk.Label(overview2, text="Altitude")
overview2.add(altitudeLabel)
altitude = tk.Label(overview2, text="0")
overview2.add(altitude)
accelxLabel = tk.Label(overview2, text="AccelX")
overview2.add(accelxLabel)
accelx = tk.Label(overview2, text="0")
overview2.add(accelx)
accelyLabel = tk.Label(overview2, text="AccelY")
overview2.add(accelyLabel)
accely = tk.Label(overview2, text="0")
overview2.add(accely)
accelzLabel = tk.Label(overview2, text="AccelZ")
overview2.add(accelzLabel)
accelz = tk.Label(overview2, text="0")
overview2.add(accelz)
spacer2 = tk.Label(overview2, text="")
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


interface.add(data, minsize=400)

rightInterface = tk.PanedWindow(interface, orient="vertical")
gpsBackground = tk.Frame(rightInterface)

gps = plt.figure(figsize=(3, 4))

fname = "Images/Large_Map.png"
img_extent = (-106.967445, -106.858726, 32.925260, 32.979024)
img = plt.imread(fname)

ax = plt.axes(projection=ccrs.PlateCarree())

ax.imshow(img, origin='upper', extent=img_extent, transform=ccrs.PlateCarree())

gpsCanvas = FigureCanvasTkAgg(gps, gpsBackground)
gpsCanvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
rightInterface.add(gpsBackground, minsize=300)

ax.plot([-106.967445, -106.858726], [32.925260, 32.979024],
         color='blue', linewidth=2, marker='o',
         transform=ccrs.Geodetic(),
         )


belowGPS = tk.PanedWindow(rightInterface, orient="horizontal")
clock = tk.Label(belowGPS, text="current time")
time_string = time.strftime('%H:%M:%S')
clock.config(text=time_string)
belowGPS.add(clock, minsize=290)
logo = tk.PhotoImage(file="logo.png", name="logo", format="png") #loads image
logoElement = tk.Label(belowGPS, text="logo",image=logo) #actually adds the image        
logoElement.image = logo #for some reason, needs this to actually render
belowGPS.add(logoElement,minsize=100)
rightInterface.add(belowGPS,minsize=100)
interface.add(rightInterface, minsize=400)
    
panes.add(interface, minsize=450)


statusBar = tk.PanedWindow(panes, orient="horizontal")
errors = tk.Label(statusBar, text="errors")
statusBar.add(errors)
indicators = tk.Label(statusBar, text="indicators")
statusBar.add(indicators)
panes.add(statusBar, minsize=15)

def tick():
    time_string = time.strftime('%H:%M:%S')
    clock.config(text=time_string)

def dataUpdate(i):
    
    packet = decoder.getPackets()
    print('[{}]'.format(', '.join(hex(x) for x in packet)))

    result = decoder.decodePackets(packet)

    print("The decoded data is: " + str(result[2]))
    print("The checksums is: " + hex(result[1]))
    print("The header is: " + hex(result[0]))
    print("The time stamp is: " + str(result[3]))
    print("Data corruption: " + str(result[4]))
    

def update(i):
    tick()
    data = pd.read_csv('data.csv')
    x = data['time']
    yPressure = data['pressure']
    pressureString = yPressure.iloc[-1]
    yTemperature = data['temperature']
    #temperature = yTemperature[-1]
    ySpeed = data['speed']
    #speed = ySpeed[-1]
    yAltitude = data['altitude']
    #altitude = yAltitude[-1]
    yAccel = data['accel']
    #accel = yAccel[-1]
    yLatitude = data['Latitude']
    #latitude = yLatitude[-1]
    yLongitude = data['Longitude']
    #longitude = yLongitude[-1]
    pressurePlot.cla()
    pressurePlot.plot(x,yPressure)
    temperaturePlot.cla()
    temperaturePlot.plot(x,yTemperature)
    altitudePlot.cla()
    altitudePlot.plot(x,yAltitude)
    speedPlot.cla()
    speedPlot.plot(x,ySpeed)
    accelPlot.cla()
    accelPlot.plot(x,yAccel)
    ax.cla()
    ax.imshow(img, origin='upper', extent=img_extent, transform=ccrs.PlateCarree())
    ax.plot(yLatitude,yLongitude)
    window.update()
    
#ani = FuncAnimation(plt.gcf(), dataUpdate, interval=1)
ani2 = FuncAnimation(plt.gcf(), update, interval=1)


