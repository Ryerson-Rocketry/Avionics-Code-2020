from tkinter import ttk
from tkinter import *
import tkinter as tk
from PIL import Image
from PIL import ImageTk
from itertools import count
import pandas as pd

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
latitude = tk.Label(overview1, text="0.0")
overview1.add(latitude)
longitudeLabel = tk.Label(overview1, text="Longitude")
overview1.add(longitudeLabel)
longitude = tk.Label(overview1, text="0.0")
overview1.add(longitude)
temperatureLabel = tk.Label(overview1, text="Temperature")
overview1.add(temperatureLabel)
temperature = tk.Label(overview1, text="0")
overview1.add(temperature)
pressureLabel = tk.Label(overview1, text="Pressure")
overview1.add(pressureLabel)
pressure = tk.Label(overview1, text="0.0")
overview1.add(pressure)
flighttimeLabel = tk.Label(overview1, text="Flight Time")
overview1.add(flighttimeLabel)
flighttime = tk.Label(overview1, text="0")
overview1.add(flighttime)
overview.add(overview1)

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
gpsCanvas = tk.Canvas(rightInterface, height=400,width=400)
gpsCanvas.configure(bg="white")

flightArea = Image.open("Images/Large_Map.png")
flightArea = flightArea.resize((390,370), Image.ANTIALIAS)
_flightArea = ImageTk.PhotoImage(flightArea)
gpsCanvas.create_image(0,0, image = _flightArea, anchor = NW)
rightInterface.add(gpsCanvas, minsize=300)

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
    pressurePlot.cla()
    pressurePlot.plot(x,y)

ani = FuncAnimation(plt.gcf(), testUpdate, interval=1000)

#while 1: # Replace with while in focus?
    #tick()
    #window.update()
    #dataUpdate(nextX) #Write backup every 5? minuntes
    #getSerial()
