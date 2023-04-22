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

class TelloAuto(object):
    """Wrapper class to enable the autonomous navigation."""
    def __init__(self):
        rospy.init_node('mapper_node', anonymous=False)

        try: 
            self.id = rospy.get_param('~ID')
        except KeyError:
            self.id = ''
        self.publish_prefix = "tello{}/".format(self.id)

        try:
            self.pose_topic_name = rospy.get_param('~POSE_TOPIC_NAME')
        except KeyError:
            self.pose_topic_name = '/ccmslam/PoseOutClient'+str(self.id)

        self.allow_slam_control = True

        self.lock = threading.Lock()
        self.command_pos = Pose()
        self.rotated_pos = Point()
        self.slam_pos = Point()
        self.twist_manual_control = Twist()
        self.real_world_pos = Point()
        self.delta_pos = Point()
        self.orientation_degree = Point()
        self.land = False
        self.point_command_pos = Point(0.0, 0.0, 1.0)
        self.point_command_pos_yaw = 0.0


        self.trajectory_threshold = Point(0.35, 0.35, 0.35)
        self.trajectory_orientation_threshold = Point(5, 5, 5)
        self.trajectory_list = []
        self.last_trajectory = []

        self.real_world_scale = 5.21

        self.cloud_topic_name = "/orb_slam2_mono/map_points"
        
        self.map = list()

        self.kd = Pose()
        self.kp = Pose() 
        self.lost = False
        self.isClose = False
        self.prevClose = False
        
        rospy.Subscriber(self.publish_prefix+'flight_data', FlightData, self.flightdata_callback)
        rospy.Subscriber(self.publish_prefix+'real_world_scale', Float32, self.real_world_scale_callback)
        rospy.Subscriber(self.publish_prefix+'real_world_pos', PoseStamped, self.real_world_pos_callback)
        #rospy.Subscriber(self.cloud_topic_name, PointCloud2, self.point_cloud_callback)
        rospy.Subscriber("/orb_slam2_mono/isLost", Bool, self.is_lost_callback)
        rospy.Subscriber('/distCalc/collision_warning', Bool, self.collision_handler)
        rospy.Subscriber(self.publish_prefix+'orientation', Point, self.orientation_callback)
        rospy.Subscriber('/tello/land', Empty, self.land_callback)

        self.command_pos_publisher = rospy.Publisher(self.publish_prefix+'command_pos', Pose, queue_size = 1)
        self.pub_takeoff = rospy.Publisher(self.publish_prefix+'takeoff', Empty, queue_size=1)
        self.pub_allow_slam_control = rospy.Publisher(self.publish_prefix+'allow_slam_control', Bool, queue_size=1)
        self.cmd_val_publisher = rospy.Publisher(self.publish_prefix+'cmd_vel', Twist, queue_size = 1)
        self.calibrate_real_world_scale_publisher = rospy.Publisher(self.publish_prefix+'calibrate_real_world_scale', Empty, queue_size = 1)
        self.scan_room_publisher = rospy.Publisher(self.publish_prefix+'scan_room', Bool, queue_size = 1)
        self.kd_publisher = rospy.Publisher(self.publish_prefix+'kd', Pose, queue_size = 1)
        self.kp_publisher = rospy.Publisher(self.publish_prefix+'kp', Pose, queue_size = 1)
        self.pub_mux =  rospy.Publisher('tello_mux', Int32, queue_size = 1)
        self.path_publisher = rospy.Publisher(self.publish_prefix+'path', Path, queue_size = 1)
        self.take_picure_publisher = rospy.Publisher(self.publish_prefix+'take_picure', Empty, queue_size=1)
        self.merge_coordinates_pub = rospy.Publisher(self.publish_prefix+'TransformerState', Bool, queue_size=1)
        self.pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
        
        self.publish_command()

    def nothing(self):
        rospy.loginfo("nothing")
        return
    
    def collision_handler(self, close):
        self.isClose = close.data
    
    def is_lost_callback(self, isLost):
        self.lost = isLost.data
            
    def real_world_pos_callback(self, msg):
        self.real_world_pos = msg.pose.position

    def real_world_scale_callback(self, msg):
        self.real_world_scale = float(msg.data)

    def calibrate_z_callback(self):
        self.calibrate_real_world_scale_publisher.publish()
        while(round(self.real_world_scale,2) == 5.21):
            time.sleep(0.1)

    def scan_room_left_callback(self):
        rospy.loginfo('pressed Scan Room Left!')
        self.scan_room_publisher.publish(True)

    def scan_room_right_callback(self):
        rospy.loginfo('pressed Scan Room Right!')
        self.scan_room_publisher.publish(False)


    def flightdata_callback(self, flight_data):
        self.altitude = flight_data.altitude
        #print(self.altitude)

    def kd_kp_callback(self):
        self.kd.position.x = float(self.kd_strigvar_x.get())
        self.kd.position.y = float(self.kd_strigvar_y.get())
        self.kd.position.z = float(self.kd_strigvar_z.get())
        self.kd.orientation.z = float(self.kd_strigvar_yaw.get())

        self.kp.position.x = float(self.kp_strigvar_x.get())
        self.kp.position.y = float(self.kp_strigvar_y.get())
        self.kp.position.z = float(self.kp_strigvar_z.get())
        self.kp.orientation.z = float(self.kp_strigvar_yaw.get())

        self.kd_publisher.publish(self.kd)
        self.kp_publisher.publish(self.kp)


    def trajectory_publish_callback(self):
        self.trajectory_kill = False
        trajectory_thread = threading.Thread(target=self.trajectory_thread, args=())
        trajectory_thread.start()

    def trajectory_pushup_callback(self):
        self.lock.acquire()
        self.trajectory_list.pop(0)
        self.lock.release()
        self.take_picure()

    def take_picure(self):
        self.take_picure_publisher.publish()

    def orientation_callback(self, orientation_point):
        self.orientation_degree = orientation_point

    def trajectory_kill_callback(self):
        self.trajectory_kill = True

    def load_trajectory_from_csv(self, trajectory_path):
        print(trajectory_path)
        file = open(trajectory_path, 'r')
        out = []
        # data = file.readlines()

        for line in file:
            temp_lst = line.strip().replace('\n','').split(',')
            x_y_z = [float(element) for element in temp_lst]
            out.append(x_y_z)
        file.close()
        self.trajectory_list = out



    def trajectory_thread(self):
        degree_point = Point()
        path_msg = Path()
        command_pos = Pose()
        self.lock.acquire()
        rospy.loginfo("Started trajectory_thread with {}".format(self.trajectory_list))

        for trajectory_idx, trajectory_line in enumerate(self.trajectory_list):
            pose_stamped = PoseStamped()
            pose_stamped.header.seq = trajectory_idx
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = 'world'

            pose_stamped.pose.position.x = trajectory_line[0]
            pose_stamped.pose.position.y = trajectory_line[1]
            pose_stamped.pose.position.z = trajectory_line[2]

            command_yaw = trajectory_line[3]
            degree_point.z = command_yaw
            command_pos.orientation = self.euler_point_deg_to_quatenrion(degree_point)

            pose_stamped.pose.orientation = command_pos.orientation
            path_msg.header = pose_stamped.header
            path_msg.poses.append(pose_stamped)
        self.lock.release()
        self.path_publisher.publish(path_msg)

        time.sleep(0.5)


        while not rospy.is_shutdown():
            time.sleep(0.2)
            if self.land:
                rospy.loginfo("Trajectory quit due to landing")
                return

            if self.trajectory_kill:
                rospy.loginfo("Trajectory quit due to killing command")
                return
            
            if self.lost:
                #self.pub_land.publish()
                rospy.loginfo("Trajectory quit due to losing reference")
                return
            

            if abs(command_pos.position.x - self.real_world_pos.x) < self.trajectory_threshold.x:
                if abs(command_pos.position.y - self.real_world_pos.y) < self.trajectory_threshold.y:
                    if abs(command_pos.position.z - self.real_world_pos.z) < self.trajectory_threshold.z:
                        if self.find_min_distance_in_orientation(command_yaw, self.orientation_degree.z) < self.trajectory_orientation_threshold.z:
                            self.trajectory_pushup_callback()
                            rospy.loginfo("Reached point")


            if len(self.trajectory_list) > 0:
                command_pos.position.x = self.trajectory_list[0][0]
                command_pos.position.y = self.trajectory_list[0][1]
                command_pos.position.z = self.trajectory_list[0][2]
                command_yaw = self.trajectory_list[0][3]
                rospy.loginfo(command_pos.position.x)
                rospy.loginfo(command_pos.position.y)
                rospy.loginfo(command_pos.position.z)
                degree_point.z = command_yaw
                command_pos.orientation = self.euler_point_deg_to_quatenrion(degree_point)


                self.command_pos_publisher.publish(command_pos)

            else:
                rospy.loginfo("Trajectory Finished")
                self.pub_land.publish()
                self.land_callback()
                return
            
            print(self.real_world_pos)
            print(self.orientation_degree)
        return

    def find_min_distance_in_orientation(self, ori1, ori2):
        max_val = 100000000
        if abs(ori1 - ori2) < max_val:
            error = ori1 - ori2
            max_val = abs(error)
        if abs(ori1 - ori2 + 360) < max_val:
            error = ori1 - ori2 + 360
            max_val = abs(error)
        if abs(ori1 - ori2 - 360) < max_val:
            error = ori1 - ori2 - 360
            max_val = abs(error)
        return max_val


    def angle_calc_set_callback(self):
        x = float(self.angle_calc_strigvar_x_moved.get())
        z = float(self.angle_calc_strigvar_z_moved.get())
        tan_angle = z/x
        self.angle_radian = math.atan(tan_angle)
        self.angle = self.angle_radian*180/math.pi
        self.angle_calc_strigvar_angle.set('%.4f'%(self.angle))  
        print("x={} z={} z/x={} angle={}".format(x, z, tan_angle, self.angle))

    def point_copy(self, p):
        return Point(p.x, p.y, p.z)

    def quatenrion_point_to_euler_degree(self, slam_quaternion):
        rad = self.quatenrion_point_to_euler(slam_quaternion)
        return Point(self.rad_to_deg(rad.x), self.rad_to_deg(rad.y), self.rad_to_deg(rad.z))

    def quatenrion_point_to_euler(self, orientation_point):
        return self.quaternion_to_orientation(orientation_point.x, orientation_point.y, orientation_point.z, orientation_point.w)

    def euler_point_deg_to_rad(self, point_deg):
        return Point(self.deg_to_rad(point_deg.x), self.deg_to_rad(point_deg.y), self.deg_to_rad(point_deg.z))


    def euler_point_deg_to_quatenrion(self, euler_point_deg):
        return self.euler_point_to_quatenrion(self.euler_point_deg_to_rad(euler_point_deg))

    def euler_point_to_quatenrion(self, euler_point):
        return self.orientation_to_quaternion(euler_point.x, euler_point.y, euler_point.z)

    def quaternion_to_orientation(self, x, y, z, w):
        euler_list = euler_from_quaternion([x, y, z, w])
        euler = Point()
        euler.x = euler_list[0]
        euler.y = euler_list[1]
        euler.z = euler_list[2]
        return euler

    def orientation_to_quaternion(self, pitch, roll, yaw):
        quaternion_list = quaternion_from_euler(pitch, roll, yaw)
        quaternion = Quaternion()
        quaternion.x = quaternion_list[0]
        quaternion.y = quaternion_list[1]
        quaternion.z = quaternion_list[2]
        quaternion.w = quaternion_list[3]
        # rospy.loginfo("quaternion={}".format(quaternion))
        return quaternion

    def rad_to_deg(self, rad):
        return rad / math.pi * 180.0

    def deg_to_rad(self, deg):
        return deg * math.pi / 180.0

    def publish_command(self):
        self.command_pos_publisher.publish(self.command_pos)
        time.sleep(0.2)
        self.command_pos_publisher.publish(self.command_pos)
        time.sleep(0.2)
        self.command_pos_publisher.publish(self.command_pos)

    def init_drone(self):
        raw_input()
        self.takeoff()
        self.allow_slam_control = False
        self.pub_allow_slam_control.publish(self.allow_slam_control)
        time.sleep(12)
        self.load_trajectory_from_csv('/home/droneops/Documents/TMR_2023/ROS/tello_catkin_ws/src/flock/flock_driver/src/mapper.csv')
        print("Loaded trajectory")
        self.calibrate_z_callback()
        self.allow_slam_control = True
        self.pub_allow_slam_control.publish(controller.allow_slam_control)
        self.trajectory_publish_callback()

    def stay_in_place(self):
        self.point_command_pos.x = self.real_world_pos.x
        self.point_command_pos.y = self.real_world_pos.y
        self.point_command_pos.z = self.real_world_pos.z
        self.point_command_pos_yaw = self.orientation_degree.z
        self.publish_command()

    def takeoff(self):
        self.land = False
        self.pub_takeoff.publish()

    def change_mux(self):
        self.current_mux = 1-self.current_mux
        self.pub_mux.publish(self.current_mux)

    def land_callback(self,msg):
        self.land = True
  
if __name__ == '__main__':
    controller = TelloAuto()
    controller.init_drone()
   