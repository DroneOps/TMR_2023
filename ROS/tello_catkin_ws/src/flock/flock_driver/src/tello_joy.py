#!/usr/bin/env python2
import rospy
from std_msgs.msg import Empty, UInt8, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class GamepadState:
    def __init__(self):
        self.A = False
        self.B = False
        self.X = False
        self.Y = False
        self.Start = False
        self.Select = False
        self.Sync = False
        self.L1 = False
        self.L2 = False
        self.L3 = False
        self.R1 = False
        self.R2 = False
        self.R3 = False
        self.DL = False
        self.DU = False
        self.DR = False
        self.DD = False
        self.LX = 0.  # +: left
        self.LY = 0.  # +: top
        self.RX = 0.  # +: left
        self.RY = 0.  # +: top
        self.LT = 0.  # 1.0: idle, -1.0: depressed
        self.RT = 0.  # 1.0: idle, -1.0: depressed

    def parse_ps3_usb(self, msg):
        # if len(msg.buttons) != 17 or len(msg.axes) != 6:
        #     raise ValueError('Invalid number of buttons (%d) or axes (%d)' % (
        #         len(msg.buttons), len(msg.axes)))
        self.A = msg.buttons[0]
        self.B = msg.buttons[1]
        self.X = msg.buttons[2]
        self.Y = msg.buttons[3]

        self.LX = msg.axes[0]
        self.LY = msg.axes[1]
        self.RX = msg.axes[3]
        self.RY = msg.axes[4]

        self.Start = msg.buttons[7] #Takeoff
        self.Select = msg.buttons[6] #Land

        
    def parse(self, msg):
        return self.parse_ps3_usb(msg)


class GamepadMarshallNode:
    MAX_FLIP_DIR = 7

    def __init__(self):
        # Define parameters
        self.joy_state_prev = GamepadState()
        # if None then not in agent mode, otherwise contains time of latest enable/ping
        self.agent_mode_t = None
        self.flip_dir = 0
        self.namespace = "tello/"
        # Start ROS node
        rospy.init_node('gamepad_marshall_node')

        # Load parameters
        self.agent_mode_timeout_sec = rospy.get_param(
            '~agent_mode_timeout_sec', 1.0)

        self.pub_takeoff = rospy.Publisher(
            self.namespace+'takeoff', Empty,  queue_size=1, latch=False)
        self.pub_throw_takeoff = rospy.Publisher(
            self.namespace+'throw_takeoff', Empty,  queue_size=1, latch=False)
        self.pub_land = rospy.Publisher(
            self.namespace+'land', Empty,  queue_size=1, latch=False)
        self.pub_palm_land = rospy.Publisher(
            self.namespace+'palm_land', Empty,  queue_size=1, latch=False)
        self.pub_reset = rospy.Publisher(
            self.namespace+'reset', Empty,  queue_size=1, latch=False)
        self.pub_flattrim = rospy.Publisher(
            self.namespace+'flattrim', Empty,  queue_size=1, latch=False)
        self.pub_flip = rospy.Publisher(
            self.namespace+'flip', UInt8,  queue_size=1, latch=False)
        self.pub_cmd_out = rospy.Publisher(
            self.namespace+'cmd_vel', Twist, queue_size=10, latch=False)
        self.pub_fast_mode = rospy.Publisher(
            self.namespace+'fast_mode', Bool,  queue_size=1, latch=False)
        self.sub_agent_cmd_in = rospy.Subscriber(
            self.namespace+'agent_cmd_vel_in', Twist, self.agent_cmd_cb)
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_cb)
        rospy.loginfo('Gamepad marshall node initialized')

    def agent_cmd_cb(self, msg):
        if self.agent_mode_t is not None:
            # Check for idle timeout
            if (rospy.Time.now() - self.agent_mode_t).to_sec() > self.agent_mode_timeout_sec:
                self.agent_mode_t = None
            else:
                self.pub_cmd_out.publish(msg)

    def joy_cb(self, msg):
        self.joy_state = GamepadState()
        self.joy_state.parse(msg)

        # Process emergency stop
        if not self.joy_state_prev.B and self.joy_state.B:
            self.pub_reset.publish()
            #rospy.logwarn('Issued RESET')
            return

        # Process takeoff
        if not self.joy_state_prev.Start and self.joy_state.Start:
            self.pub_takeoff.publish()
            #rospy.logwarn('Issued TAKEOFF')

        # Process throw takeoff
        if not self.joy_state_prev.DU and self.joy_state.DU:
            self.pub_throw_takeoff.publish()
            #rospy.logwarn('Issued THROW_TAKEOFF')

        # Process land
        if not self.joy_state_prev.Select and self.joy_state.Select:
            self.pub_land.publish()
            #rospy.logwarn('Issued LAND')

        # Process palm land
        if not self.joy_state_prev.DD and self.joy_state.DD:
            self.pub_palm_land.publish()
            #rospy.logwarn('Issued PALM_LAND')

        if not self.joy_state_prev.X and self.joy_state.X:
            self.pub_flattrim.publish()
            #rospy.logwarn('Issued FLATTRIM')

        if not self.joy_state_prev.Y and self.joy_state.Y:
            self.pub_flip.publish(self.flip_dir)
            #rospy.logwarn('Issued FLIP %d' % self.flip_dir)
            self.flip_dir += 1
            if self.flip_dir > self.MAX_FLIP_DIR:
                self.flip_dir = 0

        # Update agent bypass mode
        if self.joy_state.L2:
            self.agent_mode_t = rospy.Time.now()
        else:
            self.agent_mode_t = None

        # Manual control mode
        if self.agent_mode_t is None:
            if not self.joy_state_prev.R2 and self.joy_state.R2:
                self.pub_fast_mode.publish(True)
            elif self.joy_state_prev.R2 and not self.joy_state.R2:
                self.pub_fast_mode.publish(False)

            cmd = Twist()
            cmd.linear.x = self.joy_state.LY
            cmd.linear.y = self.joy_state.LX
            cmd.linear.z = self.joy_state.RY
            cmd.angular.z = self.joy_state.RX
            self.pub_cmd_out.publish(cmd)

        # Copy to previous state
        self.joy_state_prev = self.joy_state

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = GamepadMarshallNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass