from tkinter import *
from tkinter import ttk
import tkinter as tk
class Application(tk.Frame):
    #Screensize
    screenX = 800
    screenY = 500
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        master.title("Test GUI")
        self.pack()
        self.create_widgets()
    def create_widgets(self):
        #paned window?
        #organization = toolbar[options], window[graph[data notebook], map, logo],statusbar
        self.panes = tk.PanedWindow(orient="vertical", width = self.screenX,height=self.screenY)
        panes = self.panes
        panes.pack(fill="both",expand=1)
        toolbar = tk.PanedWindow(panes, orient="horizontal")
        options = tk.Button(toolbar, text="options")
        toolbar.add(options)
        options2 = tk.Button(toolbar, text="options")
        toolbar.add(options2)
        options3 = tk.Button(toolbar, text="options")
        toolbar.add(options3)
        optionsEmpty = tk.Label(toolbar) #Without this the buttons will stretch to fill the space
        toolbar.add(optionsEmpty)
        panes.add(toolbar,minsize=25)
        interface = tk.PanedWindow(panes, orient="horizontal")
        interface.config(bg="blue")
        data = tk.ttk.Notebook(interface)
        overview = tk.Label(data, text="overview")
        data.add(overview, text="Overview")
        
        #Position
        position = tk.PanedWindow(data, orient="vertical")
        latitude = tk.Label(position, text="latitute")
        position.add(latitude)
        longitude = tk.Label(position, text="longitude")
        position.add(longitude)
        data.add(position, text="Position")
        
        #Atmospheric Data
        atmos = tk.PanedWindow(data, orient="vertical")
        temperature = tk.Label(data, text="temperature")
        atmos.add(temperature)
        pressure = tk.Label(data, text="pressure")
        atmos.add(pressure)
        data.add(atmos, text="Atmospheric")
        
        #Flight data
        flight = tk.PanedWindow(data, orient="vertical")
        speed = tk.Label(flight, text="speed")
        flight.add(speed)
        altitude = tk.Label(flight, text="altitude")
        flight.add(altitude)
        time = tk.Label(flight, text="time")
        flight.add(time)
        data.add(flight, text="Flight data")
        
        #Acceleration
        accel = tk.PanedWindow(data, orient="vertical")
        accelX = tk.Label(accel, text="accelX")
        accel.add(accelX)
        accelY = tk.Label(accel, text="accelY")
        accel.add(accelY)
        accelZ = tk.Label(accel, text="accelZ")
        accel.add(accelZ)
        data.add(accel, text="Acceleration")
                
        interface.add(data, minsize=400)
        rightInterface = tk.PanedWindow(interface, orient="vertical")
        gpsCanvas = tk.Canvas(rightInterface, height=400,width=400)
        gpsCanvas.create_text(15,10, text="GPS")
        gpsCanvas.configure(bg="white")
        gps = tk.Label(rightInterface, text="gps")
        rightInterface.add(gpsCanvas, minsize=300)
        ''' belowGPS = tk.PanedWindow(rightInterface, orient="horizontal"
        currentTime = tk.Label(belowGPS, text="current time")
        belowGPS.add(currentTime, minsize=200)
        logo = tk.PhotoImage(file="logo.png", name="logo", format="png") #loads image
        logoElement = tk.Label(belowGPS, text="logo",image=logo) #actually adds the image        
        logoElement.image = logo #for some reason, needs this to actually render
        belowGPS.add(logoElement,minsize=100)
        rightInterface.add(belowGPS,minsize=100)'''
        interface.add(rightInterface, minsize=300)
    
        panes.add(interface, minsize=450)

        statusBar = tk.PanedWindow(panes, orient="horizontal")
        errors = tk.Label(statusBar, text="errors")
        statusBar.add(errors)
        indicators = tk.Label(statusBar, text="indicators")
        statusBar.add(indicators)
        panes.add(statusBar, minsize=15)
root = tk.Tk()
app = Application(master=root)
app.mainloop()