#!/usr/bin/env python

import rospy
import threading
import os
import time
import platform
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Bool, Int32, Float32
from flock_msgs.msg import Flip, FlightData
import signal
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import kbhit as kb

class Overrider(object):
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

        self.publish_command()

        #Subscribers
        rospy.Subscriber(self.publish_prefix+'flight_data', FlightData, self.flightdata_callback)

        #Publishers
        self.pub_land = rospy.Publisher(self.publish_prefix+'land', Empty, queue_size=1)

    def land_callback(self):
        self.land = True
        self.pub_land.publish()    

    def keyboard_checker(self, overrider):
        while True:
            time.sleep(0.3)
            if kb.kbhit():  # Check if a key is pressed
                keyboard_input = kb.getch()

            if keyboard_input == 'l':
                rospy.loginfo("Trajectory quit due to override to landing")
                overrider.land_callback()
                break

    def equipment_status_and_temperature_check(self, flight_data, overrider):
        time.sleep(10)
        if flight_data.equipment_ok != 0:
            overrider.land_callback()

        

if __name__ == '__main__':
    overrider = Overrider()
    thread_1 = Thread(target = overrider.equipment_status_and_temperature_check, args=(flight_data, overrider))   
    thread_2 = Thread(target = overrider.keyboard_checker, args=(overrider))
    thread_1.start()
    thread_2.start()
    overrider.land_callback() #Check if to keep in main thread or to put it into another thread
    
