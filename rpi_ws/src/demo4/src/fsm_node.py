#!/usr/bin/env python2.7
from math import pi, sqrt
import sys
import copy
import random
import copy
import rospy
import numpy as np
from std_msgs.msg import Int16
from std_srvs.srv import Trigger, TriggerResponse, Empty
from mobilerobot_msgs.msg import Collision, BeaconInfo


# FSM ID
STATE_ERROR = -2
STATE_UNKOWN = -1
STATE_STOP = 0
STATE_SCAN = 1
STATE_STRAIGHT = 2
STATE_SPIN_LEFT = 3
STATE_SPIN_RIGHT = 4
STATE_BACK = 5
STATE_RIGHT_BACK = 6
STATE_LEFT_BACK = 7
STATE_TURN_LEFT = 8
STATE_TURN_RIGHT = 9
STATE_SPIN_360 = 10
STATE_GOGOGO = 11

FSM_TRANS_INTERVAL = 0.5

motions = {'spin_left': (-95, 95),
            'spin_right': (95, -95),
            'straight': (120+60, 120+15),
            'back': (-100, -100),
            'right_back': (-120, -100),
            'left_back': (-100, -120),
            'turn_left': (80, 140),
            'turn_right': (140, 80),
            'forced_spin_right': (105, -105),
            'stop': (0, 0)}


class FSM(object):
    def __init__(self):
        self.next_state = STATE_UNKOWN
        self.cur_state  = STATE_UNKOWN
        self.past_state = STATE_UNKOWN

        # ROS parameters
        self.light_threshold  = rospy.get_param('~light_threshold', '150')
        self.max_cnt_right    = rospy.get_param('~max_cnt_right', '3')
        self.max_cnt_left     = rospy.get_param('~max_cnt_left', '3')
        self.max_cnt_straight = rospy.get_param('~max_cnt_straight', '5')
        self.gogo_threshold   = rospy.get_param('~gogo_threshold', '180')
        self.gate_num         = rospy.get_param('~gate', '2')
        if self.gate_num == 1:
            self.gogo_threshold = 700
        elif self.gate_num == 2:
            self.gogo_threshold = 800

        # ROS publisher & subscriber & service
        self.sub_collision  = rospy.Subscriber('collision_data', Collision, self.collision_cb)
        self.sub_beacon     = rospy.Subscriber('beacon_data', BeaconInfo, self.beacon_cb)
        self.sub_light      = rospy.Subscriber('light_value', Int16, self.light_cb)
        self.pub_r          = rospy.Publisher("right_pwm", Int16, queue_size = 10)
        self.pub_l          = rospy.Publisher("left_pwm",  Int16, queue_size = 10)
        self.start_srv      = rospy.Service("/start", Trigger, self.start_cb)
        self.flag_stop_srv  = rospy.Service("/stop", Trigger, self.stop_cb)

        self.flag_collision      = False
        self.flag_stop           = False
        # self.flag_getball        = False
        self.gate_data           = False
        self.ungate_data         = False
        self.cnt_motion_right    = 0
        self.cnt_motion_left     = 0
        self.cnt_motion_straight = 0
        self.cnt_motion_scan     = 0
        self.cnt_motion_gate     = 0
        
        self.light_data          = 0
        self.light_list          = []

        rospy.loginfo('FSM node ready.')


    def reset_var(self):
        # clear variable
        self.light_list = []
        self.cnt_motion_right = 0
        self.cnt_motion_left  = 0
        self.cnt_motion_straight = 0
        self.cnt_motion_scan = 0
        self.cnt_motion_gate = 0
        

    ############################ FSM Transition callback ############################
    def collision_cb(self, msg):
        if self.cur_state == STATE_STOP: return

        if msg.bottom:
            print 'Get light ball by touch sensor!'
            self.next_state = STATE_SPIN_360
            # self.flag_collision = True
            # self.reset_var()
        else:
            if msg.left and msg.right:
                self.next_state = STATE_BACK
                self.flag_collision = True
                self.reset_var()
            elif msg.left:
                self.next_state = STATE_LEFT_BACK
                self.flag_collision = True
                self.reset_var()
            elif msg.right:
                self.next_state = STATE_RIGHT_BACK
                self.flag_collision = True
                self.reset_var()
            else: self.flag_collision = False

    def beacon_cb(self, msg):
        if self.cur_state == STATE_STOP: return

        if (self.gate_num == 1):
             # print 'Gate Beacon600'
             self.gate_data = msg.beacon600
             self.ungate_data = msg.beacon1500
        elif (self.gate_num == 2):
             # print 'Gate Beacon1500'
             self.gate_data = msg.beacon1500
             self.ungate_data = msg.beacon600

    def light_cb(self, msg):
        if self.cur_state == STATE_STOP: return

        self.light_data = msg.data
        if (self.light_data < self.light_threshold):
            print 'Get light ball by touch sensor!', self.light_data
            self.next_state = STATE_SPIN_360
        

    def start_cb(self, req):
        self.flag_stop = False
        self.next_state = STATE_GOGOGO # STATE_STRAIGHT
        if self.cur_state != self.next_state and self.cur_state != STATE_ERROR:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            self.next_state = self.cur_state
            success, message = (False, "Request denied")


    def stop_cb(self, req):
        self.reset_var()
        self.flag_stop = True
        self.next_state = STATE_STOP


    ############################ FSM State main task ############################
    def process(self):
        # Update FSM state
        self.past_state = self.cur_state
        self.cur_state = self.next_state

        if self.cur_state != self.past_state or True:
            # straight
            if self.cur_state == STATE_STRAIGHT:
                print 'STATE_STRAIGHT'
                self.light_list
                pwm_l, pwm_r = motions['straight']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)
                rospy.sleep(0.5)
                self.cnt_motion_straight += 1
                if self.cnt_motion_straight == self.max_cnt_straight:
                    self.cnt_motion_straight = 0
                    self.next_state = STATE_SCAN if (not self.flag_collision and not self.flag_stop) else self.next_state
            
            elif self.cur_state == STATE_GOGOGO:
                print 'STATE_GOGOGO'
                self.light_list
                pwm_l, pwm_r = motions['straight']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)
                rospy.sleep(2.0)
                self.cnt_motion_straight += 1
                if self.cnt_motion_straight == self.max_cnt_straight:
                    self.cnt_motion_straight = 0
                    self.next_state = STATE_SCAN if (not self.flag_collision and not self.flag_stop) else self.next_state

            # scan
            elif self.cur_state == STATE_SCAN:
                rospy.sleep(0.2)
                print 'STATE_SCAN'
                self.light_list.append(self.light_data)

                # if the light value is smaller than gogo_threshold
                if self.light_list[-1] <= self.gogo_threshold:
                    print 'gogogo'
                    pwm_l, pwm_r = motions['straight']
                    self.pub_l.publish(pwm_l)
                    self.pub_r.publish(pwm_r)
                    rospy.sleep(1)
                    return


                self.cnt_motion_scan += 1
                if self.cnt_motion_scan == 1:
                    pwm_l, pwm_r = motions['spin_right']
                    self.pub_l.publish(pwm_l)
                    self.pub_r.publish(pwm_r)
                    rospy.sleep(0.2)
                elif self.cnt_motion_scan > 1 and self.cnt_motion_scan <= 5:
                    pwm_l, pwm_r = motions['spin_left']
                    self.pub_l.publish(pwm_l)
                    self.pub_r.publish(pwm_r)
                    rospy.sleep(0.1)

                elif self.cnt_motion_scan == 6:
                    self.light_list.pop(0)
                    print self.light_list
                    idx = np.argmin(self.light_list)
                    mininum = self.light_list[idx]

                    pwm_l, pwm_r = motions['spin_right']
                    self.pub_l.publish(pwm_l)
                    self.pub_r.publish(pwm_r)
                    rospy.sleep(0.1*(self.cnt_motion_scan-idx-1))

                    # gogogo
                    if mininum <= self.gogo_threshold:
                        pwm_l, pwm_r = motions['straight']
                        self.pub_l.publish(pwm_l)
                        self.pub_r.publish(pwm_r)
                        rospy.sleep(1)

                    self.light_list = []
                    self.cnt_motion_scan = 0
                    self.next_state = STATE_STRAIGHT if (not self.flag_collision and not self.flag_stop) else self.next_state


            # spin right
            elif self.cur_state == STATE_SPIN_RIGHT:
                rospy.sleep(0.2)
                print 'STATE_SPIN_RIGHT'
                self.light_list.append(self.light_data)

                # if the light value is smaller than gogo_threshold
                if self.light_list[-1] <= self.gogo_threshold:
                    print 'gogogo'
                    pwm_l, pwm_r = motions['straight']
                    self.pub_l.publish(pwm_l)
                    self.pub_r.publish(pwm_r)
                    rospy.sleep(1)
                    return

                 
                pwm_l, pwm_r = motions['spin_right']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)

                self.cnt_motion_right += 1
                if self.cnt_motion_right == self.max_cnt_right:
                    print self.light_list
                    idx = np.argmin(self.light_list)
                    mininum = self.light_list[idx]
                    print idx, 'has minimum'

                    # turn reverse direction
                    pwm_l, pwm_r = motions['spin_left']
                    self.pub_l.publish(pwm_l)
                    self.pub_r.publish(pwm_r)
                    rospy.sleep(0.25*(self.max_cnt_right - idx))

                    # gogogo
                    if mininum <= self.gogo_threshold:
                        pwm_l, pwm_r = motions['straight']
                        self.pub_l.publish(pwm_l)
                        self.pub_r.publish(pwm_r)
                        rospy.sleep(1)

                    self.light_list = []
                    self.cnt_motion_right = 0
                    self.next_state = STATE_STRAIGHT if (not self.flag_collision and not self.flag_stop) else self.next_state
            
            # spin left
            elif self.cur_state == STATE_SPIN_LEFT:
                rospy.sleep(0.2)
                print 'STATE_SPIN_LEFT'
                pwm_l, pwm_r = motions['spin_left']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)

                self.light_list.append(self.light_data)
                self.cnt_motion_left += 1
                if self.cnt_motion_left == self.max_cnt_left:
                    print self.light_list
                    idx = np.argmin(self.light_list)
                    mininum = self.light_list[idx]
                    print idx, 'has minimum'


                    # turn reverse direction
                    pwm_l, pwm_r = motions['spin_right']
                    self.pub_l.publish(pwm_l)
                    self.pub_r.publish(pwm_r)
                    rospy.sleep(0.22*(self.max_cnt_left - idx))

                    # gogogo
                    if mininum <= self.gogo_threshold:
                        pwm_l, pwm_r = motions['straight']
                        self.pub_l.publish(pwm_l)
                        self.pub_r.publish(pwm_r)
                        rospy.sleep(1)

                    self.light_list = []
                    self.cnt_motion_left = 0
                    self.next_state = STATE_STRAIGHT if (not self.flag_collision and not self.flag_stop) else self.next_state

            # spin 360
            elif self.cur_state == STATE_SPIN_360:
                rospy.sleep(0.2)
                print 'STATE_SPIN_360, find goal {}'.format(self.gate_num)

                self.cnt_motion_gate += 1
                if self.ungate_data:
                    pwm_l, pwm_r = motions['forced_spin_right']
                    self.pub_l.publish(pwm_l)
                    self.pub_r.publish(pwm_r)
                    rospy.sleep(0.5)

                elif self.cnt_motion_gate > 12: # about 2 round circle
                    self.cnt_motion_gate = 0
                    pwm_l, pwm_r = motions['straight']
                    self.pub_l.publish(pwm_l)
                    self.pub_r.publish(pwm_r)
                    rospy.sleep(0.2)

                elif self.gate_data and not self.ungate_data:
                    pwm_l, pwm_r = motions['straight']
                    self.pub_l.publish(pwm_l)
                    self.pub_r.publish(pwm_r)
                    print 'moving forward to goal!!!'
                    self.cnt_motion_gate = 0
                    rospy.sleep(1)
                else:
                    pwm_l, pwm_r = motions['forced_spin_right']
                    self.pub_l.publish(pwm_l)
                    self.pub_r.publish(pwm_r)
                 

                self.gate_data = False
                self.ungate_data = False
                self.next_state = STATE_SPIN_360 if (not self.flag_collision and not self.flag_stop) else self.next_state

            # turn
            elif self.cur_state == STATE_TURN_RIGHT:
                print 'STATE_TURN_RIGHT'
                pwm_l, pwm_r = motions['turn_right']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)
                rospy.sleep(0.5)
                self.next_state = STATE_SPIN_RIGHT if (not self.flag_collision and not self.flag_stop) else self.next_state
            elif self.cur_state == STATE_TURN_LEFT:
                print 'STATE_TURN_LEFT'
                pwm_l, pwm_r = motions['turn_left']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)
                rospy.sleep(0.5)
                self.next_state = STATE_STRAIGHT if (not self.flag_collision and not self.flag_stop) else self.next_state

            # back
            elif self.cur_state == STATE_BACK:
                print 'STATE_BACK'
                pwm_l, pwm_r = motions['back']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)
                self.next_state = STATE_TURN_RIGHT if (not self.flag_collision and not self.flag_stop) else self.next_state

            elif self.cur_state == STATE_LEFT_BACK:
                print 'STATE_LEFT_BACK'
                pwm_l, pwm_r = motions['left_back']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)
                self.next_state = STATE_TURN_RIGHT if (not self.flag_collision and not self.flag_stop) else self.next_state

            elif self.cur_state == STATE_RIGHT_BACK:
                print 'STATE_RIGHT_BACK'
                pwm_l, pwm_r = motions['right_back']
                self.pub_l.publish(pwm_l)
                self.pub_r.publish(pwm_r)
                self.next_state = STATE_TURN_LEFT if (not self.flag_collision and not self.flag_stop) else self.next_state
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

    # r = rospy.Rate(2)
    while not rospy.is_shutdown():
        node.pub_l.publish(Int16())
        node.pub_r.publish(Int16())
        rospy.sleep(0.02)
        node.process()
        rospy.sleep(FSM_TRANS_INTERVAL-0.02)
        # r.sleep()
    rospy.spin()
