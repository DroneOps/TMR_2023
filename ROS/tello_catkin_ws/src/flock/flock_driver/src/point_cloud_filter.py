#!/usr/bin/env python

import rospy
import threading
import time
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Empty, Bool, Int32, Float32
from flock_msgs.msg import Flip, FlightData
import math
import ros_numpy
import numpy as np
import csv
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion, quaternion_from_euler




class PointCloudFilter(object):
    def __init__(self):
        rospy.init_node('drone_distance', anonymous=False)
        try: 
            self.id = rospy.get_param('~ID')
        except KeyError:
            self.id = ''
        self.publish_prefix = "distCalc{}/".format(self.id)

        try:
            self.pose_topic_name = rospy.get_param('~POSE_TOPIC_NAME')
        except KeyError:
            self.pose_topic_name = '/ccmslam/PoseOutClient'+str(self.id)

        self.cloud_topic_name = "/orb_slam2_mono/map_points"
        self.map = np.array([],[])
        self.real_world_pos = Point(0,0,0)
        self.real_world_scale = 1

        #Filter_Params
        self.drone_path_radius = 1
        
        #Subscribers
        rospy.Subscriber('/tello/real_world_scale', Float32, self.real_world_scale_callback)
        rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.real_world_pos_callback)
        rospy.Subscriber(self.cloud_topic_name, PointCloud2, self.point_cloud_callback)

    def real_world_pos_callback(self, msg):
        self.real_world_pos = msg.pose.position
    
    def point_cloud_callback(self, raw_map):
        self.map = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(raw_map)

    def real_world_scale_callback(self, msg):
        self.real_world_scale = float(msg.data)
        
    def filter_points(self, temp_map):
        if temp_map.shape[0] > 0:
            temp_map = temp_map[np.all(temp_map != 0, axis=1)]
            if temp_map.shape[0] > 0:
                for i in range(temp_map.shape[0]-1,0,-1):
                    #print(temp_map.shape)
                    if(abs(temp_map[i,0])>self.drone_path_radius or abs(temp_map[i,1])>self.drone_path_radius or abs(temp_map[i,2])>self.drone_path_radius):
                        temp_map = np.delete(temp_map, (i), axis=0)
            time.sleep(0.2)
        return temp_map
    
    def map2csv(self, path):
            while not rospy.is_shutdown():
                temp_map = self.map
                # if temp_map.shape[0] > 0:
                #     temp_map = temp_map*self.real_world_scale
                # filtered_map = self.filter_points(temp_map)
            path_file = open(path, 'w')
            writer = csv.writer(path_file)
            for coord in temp_map:
                writer.writerow(coord)

if __name__ == '__main__':
    calculator = PointCloudFilter()
    calculator.map2csv('/home/droneops/Documents/TMR_2023/ROS/tello_catkin_ws/src/flock/flock_driver/scripts/pc.pts')