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
    
    def relative2absolute_speed(self,cmd_vel, angular_pose):
        linear_speed = [
            np.cos(angular_pose) * cmd_vel.linear.x - np.sin(angular_pose) * cmd_vel.linear.y,
            np.sin(angular_pose) * cmd_vel.linear.x + np.cos(angular_pose) * cmd_vel.linear.y,
            cmd_vel.linear.z]
        return linear_speed
    
    def calc_speed(self,linear_pos, prev_pos, delta_t):
        linear_vel = [
            (linear_pos.x - prev_pos[0])/delta_t,
            (linear_pos.y - prev_pos[1])/delta_t,
            (linear_pos.z - prev_pos[2])/delta_t,]
        return linear_vel

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
        prev_position = [0, 0, 0]
        linear_vel = [0,0,0]
        path_file = open('/home/pablo/Documents/Datasets/orb_slam_dataset/'+file_name, 'w')
        writer = csv.writer(path_file)
        writer.writerow(['time',
                         'linear_pos_x', 'linear_pos_y', 'linear_pos_z', 
                         'angular_pos_x', 'angular_pos_y', 'angular_pos_z', 
                         'cmd_vel_x' , 'cmd_vel_y', 'cmd_vel_z',
                         'linear_vel_x', 'linear_vel_y', 'linear_vel_z',
                         'angular_vel_x' , 'angular_vel_y', 'angular_vel_z' ])
        while not rospy.is_shutdown():
            linear_control_speed = self.relative2absolute_speed(self.cmd_vel, self.real_world_pos.orientation.z)
            row = [rospy.get_rostime().secs,
                   self.real_world_pos.position.x,self.real_world_pos.position.y,self.real_world_pos.position.z,
                   self.real_world_pos.orientation.x,self.real_world_pos.orientation.y,self.real_world_pos.orientation.z,
                   linear_control_speed[0],linear_control_speed[1],linear_control_speed[2],
                   linear_vel[0], linear_vel[1], linear_vel[2],
                   self.cmd_vel.angular.x,self.cmd_vel.angular.y,self.cmd_vel.angular.z]
            writer.writerow(row)
            prev_position = [self.real_world_pos.position.x, self.real_world_pos.position.y, self.real_world_pos.position.z]
            time.sleep(0.05)
            linear_vel = self.calc_speed(self.real_world_pos.position,prev_position,0.05)

if __name__ == '__main__':
    controller = DataLogger()
    controller.generate_logs("teleop_testing.txt")

   
