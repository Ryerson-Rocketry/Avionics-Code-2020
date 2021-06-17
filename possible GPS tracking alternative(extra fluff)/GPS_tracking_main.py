from geopy import distance
import time
import serial
import numpy as np
import cv2
import math
from math import radians
import pandas as pd
import numpy as np
import random
#geopy documentation: https://geopy.readthedocs.io/en/stable/

import GPS_tracking_path
import csv
from shapely.geometry import Point, Polygon

#using test values simulating received w/ rfd and decoded:
#********** NOTE: it seems that changing lat and long pt by 1 is around 0.5km **********

Spaceport_location = [32.988789,-106.975174]
[minCompLat,maxCompLat] = [Spaceport_location[0]-0.5,Spaceport_location[0]+0.5] 

[minCompLong, maxCompLong] = [Spaceport_location[1]-0.5, Spaceport_location[1]+0.5]

avionicsBay_latitude = random.uniform(minCompLat, maxCompLat)
avionicsBay_longitude = random.uniform(minCompLong, maxCompLong)

#get ground station GPS data:

with serial.Serial('COM4', 9600, timeout=1) as ser:
    GPS_NMEA = ser.read     # read up to ten bytes (timeout)
    ser.flush()
    print(GPS_NMEA)
    
