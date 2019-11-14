#!/usr/bin/env python2.7
from math import pi, sqrt
import sys
import copy
import random
import copy
import rospy
from std_msgs.msg import Int16
from std_srvs.srv import Trigger, TriggerResponse, Empty
from mobilerobot_msgs.msg import Collision


# FSM ID
STATE_ERROR = -2
STATE_UNKOWN = -1
STATE_STOP = 0
STATE_RECOGNITION = 1
STATE_STRAIGHT = 2
STATE_TURN_LEFT = 3
STATE_TURN_RIGHT = 4
STATE_BACK = 5
STATE_RIGHT_BACK = 6
STATE_LEFT_BACK = 7


motions = {'turn_left': (0, 100),
        'turn_right': (100, 0),
        'straight': (100, 100),
        'back': (-100, -100),
        'right_back': (-120, -100),
        'left_back': (-100, -120),
        'stop': (0, 0)}


class FSM(object):
    def __init__(self):
        self.next_state = STATE_UNKOWN
        self.cur_state = STATE_UNKOWN
        self.past_state = STATE_UNKOWN

        # ROS parameters
        self.light_threshold = rospy.get_param('~light_threshold', '150')
        self.max_cnt_right = rospy.get_param('~max_cnt_right', '3')
        self.max_cnt_left = rospy.get_param('~max_cnt_left', '3')
        self.max_cnt_straight = rospy.get_param('~max_cnt_straight', '5')

        # ROS publisher & subscriber & service
        self.sub_collision = rospy.Subscriber('collision_data', Collision, self.collision_cb)
        self.sub_light = rospy.Subscriber('light_value', Int16, self.light_cb)
        self.pub_r = rospy.Publisher("right_pwm", Int16, queue_size = 10)
        self.pub_l = rospy.Publisher("left_pwm",  Int16, queue_size = 10)
        self.start_srv = rospy.Service("/start", Trigger, self.start_cb)
        self.stop_srv = rospy.Service("/stop", Trigger, self.stop_cb)

        self.flag_collision = False
        self.cnt_motion_right = 0
        self.cnt_motion_left = 0
        self.cnt_motion_straight = 0
        self.light_list = []
        self.light_data = 0

        rospy.loginfo('FSM node ready.')

    ############################ FSM Transition callback ############################
    def collision_cb(self, msg):
        if self.cur_state == STATE_STOP:
            return

        # clear variable
            self.light_list = []
            self.cnt_motion_right = 0
            self.cnt_motion_left = 0

        if msg.bottom:
            rospy.wait_for_service('stop', timeout=3)
            stop_action = rospy.ServiceProxy('stop', Empty)
            stop_action()
        else:
            if msg.left and msg.right:
                self.next_state = STATE_BACK
                self.flag_collision = True
            elif msg.left:
                self.next_state = STATE_LEFT_BACK
                self.flag_collision = True
            elif msg.right:
                self.next_state = STATE_RIGHT_BACK
                self.flag_collision = True
            else: self.flag_collision = False

    def light_cb(self, msg):
        self.light_data = msg.data
        if (self.light_data < self.light_threshold):
            rospy.wait_for_service('stop', timeout=3)
            close_action = rospy.ServiceProxy('stop', Empty)
            close_action() 
        

    def start_cb(self, req):
        self.next_state = STATE_STRAIGHT
        if self.cur_state != self.next_state and self.cur_state != STATE_ERROR:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            self.next_state = self.cur_state
            success, message = (False, "Request denied")

    def stop_cb(self, req):
        self.next_state = STATE_STOP
        if self.cur_state != self.next_state and self.cur_state != STATE_ERROR:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            self.next_state = self.cur_state
            success, message = (False, "Request denied")

    def calibrate_cb(self, req):
        self.next_state = STATE_CALIBRATION
        if self.cur_state != self.next_state and self.cur_state != STATE_ERROR:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            self.next_state = self.cur_state
            success, message = (False, "Request denied")


    ############################ FSM State main task ############################
    def process(self):
        # Update FSM state
        self.past_state = self.cur_state
        self.cur_state = self.next_state

        if self.cur_state != self.past_state or True:
            if self.cur_state == STATE_STRAIGHT:
                print 'STATE_STRAIGHT'
                self.light_list
                pwm_l, pwm_r = motions['straight']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)
                self.cnt_motion_straight += 1
                if self.cnt_motion_straight == self.max_cnt_straight:
                    self.cnt_motion_straight = 0
                    self.next_state = STATE_TURN_RIGHT if not self.flag_collision else self.next_state

            elif self.cur_state == STATE_TURN_RIGHT:
                print 'STATE_TURN_RIGHT'
                pwm_l, pwm_r = motions['turn_right']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)

                self.light_list.append(self.light_data)
                self.cnt_motion_right += 1
                if self.cnt_motion_right == self.max_cnt_right:
                    print self.light_list
                    self.light_list = []
                    self.cnt_motion_right = 0
                    self.next_state = STATE_TURN_LEFT if not self.flag_collision else self.next_state

            elif self.cur_state == STATE_TURN_LEFT:
                print 'STATE_TURN_LEFT'
                pwm_l, pwm_r = motions['turn_left']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)

                self.light_list.append(self.light_data)
                self.cnt_motion_left += 1
                if self.cnt_motion_left == self.max_cnt_left:
                    print self.light_list
                    self.light_list = []
                    self.cnt_motion_left = 0
                    self.next_state = STATE_STOP if not self.flag_collision else self.next_state

            # back
            elif self.cur_state == STATE_BACK:
                print 'STATE_BACK'
                pwm_l, pwm_r = motions['back']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)
                self.next_state = STATE_STRAIGHT if not self.flag_collision else self.next_state
            elif self.cur_state == STATE_LEFT_BACK:
                print 'STATE_LEFT_BACK'
                pwm_l, pwm_r = motions['left_back']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)
                self.next_state = STATE_STRAIGHT if not self.flag_collision else self.next_state
            elif self.cur_state == STATE_RIGHT_BACK:
                print 'STATE_RIGHT_BACK'
                pwm_l, pwm_r = motions['right_back']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)
                self.next_state = STATE_STRAIGHT if not self.flag_collision else self.next_state
            # stop
            elif self.cur_state == STATE_STOP:
                print 'STATE_STOP'
                pwm_l, pwm_r = motions['stop']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)


            
    def shutdown_cb(self):
        rospy.loginfo("Node shutdown")


if __name__ == '__main__':
    rospy.init_node('fsm_node', anonymous=False)
    node = FSM()
    rospy.on_shutdown(node.shutdown_cb)

    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        node.pub_l.publish(Int16())
        node.pub_r.publish(Int16())
        node.process()
        r.sleep()
    rospy.spin()
