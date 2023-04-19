#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Bool, Int32, Float32
from flock_msgs.msg import Flip, FlightData
import numpy as np
import csv
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class DataLogger(object):
    """Wrapper class to enable the autonomous navigation."""
    def __init__(self):
        rospy.init_node('tello_auto', anonymous=False)

        try: 
            self.id = rospy.get_param('~ID')
        except KeyError:
            self.id = ''
        self.publish_prefix = "tello{}/".format(self.id)

        try:
            self.pose_topic_name = rospy.get_param('~POSE_TOPIC_NAME')
        except KeyError:
            self.pose_topic_name = '/ccmslam/PoseOutClient'+str(self.id)

        self.real_world_pos = Pose()
        self.cmd_vel = Twist()
        self.real_world_scale = 0
        self.flight_data = 0
        self.baterry = 0
    
        rospy.Subscriber(self.publish_prefix+'flight_data', FlightData, self.flightdata_callback)
        rospy.Subscriber(self.publish_prefix+'real_world_scale', Float32, self.real_world_scale_callback)
        rospy.Subscriber(self.publish_prefix+'real_world_pos', PoseStamped, self.real_world_pos_callback)
        rospy.Subscriber(self.publish_prefix+'cmd_vel', Twist, self.cmd_vel_callback)
    
    
    def real_world_pos_callback(self, msg):
        self.real_world_pos = msg.pose
        

    def real_world_scale_callback(self, msg):
        self.real_world_scale = float(msg.data)

    def flightdata_callback(self, flight_data):
        self.altitude = flight_data.altitude
        self.baterry = flight_data.battery_percent
    
    def cmd_vel_callback(self, cmdVel):
        self.cmd_vel = cmdVel

    def generate_logs(self,file_name):
        path_file = open('/home/droneops/Documents/Datasets/OrbSLAM_dataset/'+file_name, 'w')
        writer = csv.writer(path_file)
        writer.writerow(['Time',
                         'Linear Pos X', 'Linear  Pos Y', 'Linear Pos Z', 
                         'Angular Pos X', 'Angular Pos Y', 'Angular Pos Z', 
                         'Lineal Vel X' , 'Lineal Vel Y', 'Lineal Vel Z',
                         'Angular Vel X' , 'Angular Vel Y', 'Angular Vel Z' ])
        while not rospy.is_shutdown():
            row = [rospy.get_rostime().secs,
                   self.real_world_pos.position.x,self.real_world_pos.position.y,self.real_world_pos.position.z,
                   self.real_world_pos.orientation.x,self.real_world_pos.orientation.y,self.real_world_pos.orientation.z,
                   self.cmd_vel.linear.x,self.cmd_vel.linear.y,self.cmd_vel.linear.z,
                   self.cmd_vel.angular.x,self.cmd_vel.angular.y,self.cmd_vel.angular.z,]
            writer.writerow(row)
            time.sleep(0.8)
            
    

if __name__ == '__main__':
    controller = DataLogger()
    controller.generate_logs("MapperTest.txt")

   