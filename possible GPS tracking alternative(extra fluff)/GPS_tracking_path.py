from geopy import distance
import time
import serial
import numpy as np
from math import radians
import numpy as np
import random
from shapely.geometry import Point, Polygon
from shapely import speedups
#geopy documentation: https://geopy.readthedocs.io/en/stable/

#class RoverDistance:
          
def FindDistance(lattitude,longitude,lattitude2,longitude2):
  Pt1 =[lattitude,longitude]
  Pt2 = [lattitude2,longitude2]
  Dist = [round( distance.distance(Pt1, Pt2).m),round((distance.distance(Pt1, Pt2).m)/100)]#gets distance (difference betw. pts)
  #print(distance.distance(GPSdata,SLC).km)
  return (Dist[0])


'''
-------------------------GEOFENCE METHODS-----------------------
1. method 1 (M1)- my geofence method
(making another array adding a threshold to each GPS point)

2. method 2(M2) - haversine method, from: https://stackoverflow.com/questions/4913349/haversine-formula-in-python-bearing-and-distance-between-two-gps-points
3. method 3(M3)- using Shapely method , from: https://www.reddit.com/r/learnpython/comments/6ygu1a/how_does_geofencing_work_how_to_determine_if_a/
4. method 4(M4)- picket library: https://github.com/sam-drew/picket
5. method 5(M5)- geopy library: https://geopy.readthedocs.io/en/stable/#module-geopy.distance

'''
#-------------------------------------------------------------------------
#------------------my own method:


'''
CompLocation = [51.463052, -112.694881]
********** NOTE: it seems that changing lat and long pt by 1 is around 0.5km **********
maxCompLat = CompLocation[0]+0.5
minCompLat = CompLocation[0]-0.5
maxCompLong = CompLocation[1]+0.5
minCompLong = CompLocation[1]-0.5
'''
def Geofencing (lattitude,longitude,lat_threshold,long_threshold):
  #Make sure before running function that Lat and long threshold inputted is within reason (if the whole map is 2km x 2km then threshold must be less than 2km) 
  '''
  # trying to optimize code by using a list as a function input:
  Location =[]
  for ijk in len(GPS_threshold):
  Location.append(GPS_threshold[ijk])
  '''
  threshold = [lat_threshold,long_threshold]

  GPSpt =[lattitude,longitude]
  
  '''
  ser = serial.Serial('/dev/ttyACM0', 9600)#assigning to port 
  Position=ser.readline()#assigning to variable

  if gps is set up uncomment ser and Position
  '''
  #shapely method:https://automating-gis-processes.github.io/2017/lessons/L3/point-in-polygon.html
  

  speedups.disable() #from: https://stackoverflow.com/questions/62075847/using-qgis-and-shaply-error-geosgeom-createlinearring-r-returned-a-null-pointer
  
  
  P1 = (lattitude-lat_threshold)
  P2 = (longitude+ long_threshold)
  P3 = (lattitude + lat_threshold)
  P4 = (longitude + long_threshold)
  P5 = (lattitude-lat_threshold)
  P6 = (longitude-long_threshold)
  P7 = (lattitude + lat_threshold)
  P8 =(longitude-long_threshold)
  # Create a Polygon from the coordinates
  #print(P1,P2,P3,P4,P5,P6,P7,P8)
  poly = Polygon([(P1,P2),(P3, P4),(P5, P6), (P7,P8),(P1,P2)]) 
  
  # template is: [TopLeft,TopRight,BottomLeft,BottomRight]
  #print("using M2, The polygon fence is:" +str(poly)+"\n")
  
  CurrentGeoFencePts =[] # template is: [TopLeft,TopRight,BottomLeft,BottomRight]
  CurrentGeoFencePts.append([P1,P2])
  CurrentGeoFencePts.append([P3,P4])
  CurrentGeoFencePts.append([P5,P6])
  CurrentGeoFencePts.append([P7,P8])


  #print("Geofence method 1, the geofence is:"+ str(CurrentGeoFencePts)+"\n")
 
  return (CurrentGeoFencePts,poly)
#-------------------------------------------------------------------------
#-------------------------------------------------------------------------

#haversine method: converting cartesian to gps lat/long ("OPTIONAL METHOD")
def haversine (lon1, lat1, lon2, lat2):
  from math import radians, cos, sin, asin, sqrt
  """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
  """
  # convert decimal degrees to radians 
  lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

  # haversine formula 
  dlon = lon2 - lon1 
  dlat = lat2 - lat1 
  a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
  c = 2 * asin(sqrt(a)) 
  r = 6371 # Radius of earth in kilometers. Use 3956 for miles
  return c * r




          
            


