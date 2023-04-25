#!/usr/bin/env python

import rospy
import threading
import time
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Bool, Int32, Float32
from flock_msgs.msg import Flip, FlightData
import sys

class OverrideNode(object):
    def __init__(self):
        rospy.init_node('override_node', anonymous=False)

        try: 
            self.id = rospy.get_param('~ID')
        except KeyError:
            self.id = ''
        self.publish_prefix = "tello{}/".format(self.id)

        try:
            self.pose_topic_name = rospy.get_param('~POSE_TOPIC_NAME')
        except KeyError:
            self.pose_topic_name = '/ccmslam/PoseOutClient'+str(self.id)

        self.drone_status = 0
        #Subscribers
        rospy.Subscriber('/tello/flight_data', FlightData, self.flightdata_callback)
        #Publishers
        self.pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
        
    def flightdata_callback(self,msg):
        self.drone_status = msg.equipment_ok
    
    def land_callback(self):
        self.pub_land.publish()    

    def keyboard_checker(self):
        while not rospy.is_shutdown():
            raw_input()
            rospy.loginfo("Trajectory quit due to override to landing")
            self.land_callback()
            time.sleep(3)

    def equipment_status_and_temperature_check(self):
        while not rospy.is_shutdown():
            time.sleep(0.2)
            if self.drone_status != 0:
                rospy.loginfo("Trajectory quit due to equipment failure")
                self.land_callback()
        
if __name__ == '__main__':
    print("--OVERRIDER CONSOLE--")
    print(" ")
    print("Press any key to kill trajectory")
    security_node = OverrideNode()
    auto_thread = threading.Thread(target = security_node.equipment_status_and_temperature_check, args=())   
    key_thread = threading.Thread(target = security_node.keyboard_checker, args=())

    key_thread.start()
    auto_thread.start()
   
