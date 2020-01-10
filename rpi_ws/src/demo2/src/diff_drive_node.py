#!/usr/bin/env python

'''
  PID controller for differential-drive robot 
  Things to do:
     and publish odom topic -> called by timer with 100Hz
   * Subscribe to twist topic, calculate desired linear velocities of two wheels and use
     PID controller to chase the setpoint, produce pwm value for motors -> called by callback
'''

import serial
import rospy
import tf

from math import sin, cos, isnan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Int16
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import Joy

WIDTH    = 0.172
GAIN     = 1.0
K        = 27.0
LIMIT    = 1.0
RADIUS   = 0.032
TRIM     = -0.24 #-0.24
MAX_PWM  = 100 #200
MIN_PWM  = 80  # minimum value to launch motor

class DiffController(object):
    def __init__(self):
        self.mode_joystick = True

        # Desired velocities
        self.v_d = 0
        self.w_d = 0

        # ROS Subscriber and publisher
        self.pub_right = rospy.Publisher('right_pwm', Int16, queue_size = 1)
        self.pub_left = rospy.Publisher('left_pwm', Int16, queue_size = 1)
        self.sub_cmd  = rospy.Subscriber('cmd_vel', Twist, self.cmd_cb,  queue_size = 1)
        self.sub_auto_cmd  = rospy.Subscriber('auto_cmd_vel', Twist, self.auto_cmd_cb,  queue_size = 1)

        # ROS Parameters
        self.k = rospy.get_param('~k', K)
        self.gain = rospy.get_param('~gain', GAIN)
        self.trim = rospy.get_param('~trim', TRIM)
        self.radius = rospy.get_param('~radius', RADIUS)
        self.limit = rospy.get_param('~limit', LIMIT)

        # ROS Timer
        rospy.Timer(rospy.Duration(1 / 5.0), self.timer_cb) # 5Hz
        rospy.loginfo('[%s] Initialized'  %(rospy.get_name()))

        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)


    def timer_cb(self, event):
        # Assuming the same motor spec constants k for both motors
        k_r = self.k
        k_l = self.k

        # Adjusting k by gain and trim
        k_r_inv = (self.gain + self.trim) / k_r
        k_l_inv = (self.gain - self.trim) / k_l

        omega_r = (self.v_d + 0.5 * self.w_d) / self.radius
        omega_l = (self.v_d - 0.5 * self.w_d) / self.radius

        # conversion from motor rotation rate to duty cycle
        u_r = omega_r * k_r_inv
        u_l = omega_l * k_l_inv

        # Limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = max(min(u_r, self.limit), -self.limit)
        u_l_limited = max(min(u_l, self.limit), -self.limit)

        # print "u_r: " + str(u_r_limited) + ", u_l: " + str(u_l_limited)
        self.motor_motion(u_r_limited * (MAX_PWM-MIN_PWM) + MIN_PWM*u_r_limited/abs(u_r_limited+1e-07), \
                              u_l_limited * (MAX_PWM-MIN_PWM) + MIN_PWM*u_l_limited/abs(u_l_limited+1e-07))

    # sub_cmd callback, get desired linear and angular velocity
    def cmd_cb(self, msg):
        if self.mode_joystick:
            self.v_d = msg.linear.x
            self.w_d = msg.angular.z
            rospy.loginfo('joy mode, vel: {:.3f}, omega: {:.3f}'.format(self.v_d, self.w_d))
    
    def auto_cmd_cb(self, msg):
        if not self.mode_joystick:
            self.v_d = msg.linear.x
            self.w_d = msg.angular.z
            rospy.loginfo('auto mode, vel: {:.3f}, omega: {:.3f}'.format(self.v_d, self.w_d))
        


    # Send command to motors
    # pwm_r: right motor PWM value
    # pwm_l: left motor PWM value
    def motor_motion(self, pwm_r, pwm_l):
        self.pub_right.publish(int(pwm_r))
        self.pub_left.publish(int(pwm_l))
        #print('(pwm_l, pwm_r) = ({:2f}, {:2f})'.format(pwm_l, pwm_r))

    def cbJoy(self, joy_msg):
        if (joy_msg.buttons[6] == 1): #The back button, joystick control
            rospy.loginfo('Joystick_control = True')
            self.mode_joystick = True
            #self.pub_right.publish(Int16())
            #self.pub_left.publish(Int16())
        elif (joy_msg.buttons[7] == 1): #the start button
            rospy.loginfo('Auto_control = True')
            self.mode_joystick = False
            #self.pub_right.publish(Int16())
            #self.pub_left.publish(Int16())


    # Shutdown function, call when terminate
    def shutdown(self):
        self.sub_cmd.unregister()
        rospy.sleep(0.1)
        self.pub_right.publish(Int16())
        self.pub_left.publish(Int16()) 

if __name__ == '__main__':
    rospy.init_node('diff_controller_node')
    controller = DiffController()
    rospy.on_shutdown(controller.shutdown)
    rospy.spin()
