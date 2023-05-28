import os
import math
import csv
import numpy as np  

def points_circle(radius,points, height_max, height_min):
    return [(-1*round(math.cos(2*math.pi / points*x)*radius,2)+radius,
             -1*round(math.sin(2*math.pi / points*x)*radius,2),
             round((math.cos(x*0.5*math.pi)+1)*0.5*(height_max-height_min)+height_min,2),
             90-1*round(np.rad2deg(math.atan2(math.cos(2*math.pi / points*x)*radius,math.sin(2*math.pi / points*x)*radius)),2)) 
             for x in range(0,points+1)]

def write2csv(pointvec):
    path_file = open('/home/pablo/TMR_2023/ROS/tello_catkin_ws/src/flock/flock_driver/src/mapper.csv', 'w')
    writer = csv.writer(path_file)
    for coord in pointvec:
        writer.writerow(coord)

write2csv(points_circle(1,20,0.7,0.4))