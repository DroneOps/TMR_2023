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
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion, quaternion_from_euler




class DistanceCalculator(object):
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
        self.CollisionWarning = False
        self.real_world_pos = Point(0,0,0)
        self.real_world_scale = 1

        #Subscribers
        rospy.Subscriber('/tello/real_world_scale', Float32, self.real_world_scale_callback)
        rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.real_world_pos_callback)
        rospy.Subscriber(self.cloud_topic_name, PointCloud2, self.point_cloud_callback)
        #Publishers
        self.collider_pub = rospy.Publisher(self.publish_prefix+'collision_warning', Bool, queue_size=1)

    def real_world_pos_callback(self, msg):
        self.real_world_pos = msg.pose.position
    
    def point_cloud_callback(self, raw_map):
        self.map = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(raw_map)
    def real_world_scale_callback(self, msg):
        self.real_world_scale = float(msg.data)
        
    def calc_distance(self):
        while not rospy.is_shutdown():
            temp_map = self.map
            self.collider_pub.publish(self.CollisionWarning)
            if temp_map.shape[0] > 0:
                    temp_map = temp_map[np.all(temp_map > 0.1, axis=1)]
                #if temp_map.shape[0] > 0:
                    #for i in range(temp_map.shape[0]-1,0,-1):
                        #print(temp_map.shape)
                        #if(temp_map[i,1]>0.3 or temp_map[i,1]<-0.3 or temp_map[i,2]>self.real_world_pos.z + 0.5):
                            #temp_map = np.delete(temp_map, (i), axis=0)
                    if(temp_map.shape[0] > 0 and self.real_world_pos):
                        temp_map = temp_map[temp_map[:, 0].argsort()]
                        if(self.real_world_pos.x > temp_map[0, 0] * 7/16):
                            self.CollisionWarning = True
            time.sleep(0.2)
        return


if __name__ == '__main__':
    calculator = DistanceCalculator()
    calculator.calc_distance()

