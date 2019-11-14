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
STATE_UNKOWN = -1
STATE_STOP = 0
STATE_RECOGNITION = 1
STATE_ERROR = -2


motions = {'turn_left': (0, 100),
        'turn_right': (100, 0),
        'go': (100, 100),
        'back': (-100, -100),
        'right_back': (-120, -100),
        'left_back ': (-100, -120),
        'stop': (0, 0)}


class FSM(object):
    """docstring for FSM"""
    def __init__(self):
        self.next_state = STATE_UNKOWN
        self.cur_state = STATE_UNKOWN
        self.past_state = STATE_UNKOWN

        self.light_threshold = rospy.get_param('~light_threshold', '150')

        # ROS publisher & subscriber & service
        self.sub_collision = rospy.Subscriber('collision_data', Collision, self.collision_cb)
        self.sub_light = rospy.Subscriber('/arg3/light_value', Int16, self.light_cb)
        self.pub_r = rospy.Publisher("right_pwm", Int16, queue_size = 10)
        self.pub_l = rospy.Publisher("left_pwm",  Int16, queue_size = 10)
        self.start_srv = rospy.Service("start", Trigger, self.start_cb)
        self.stop_srv = rospy.Service("stop", Trigger, self.stop_cb)

    ############################ FSM Transition callback ############################
    self.collision_cb(self, msg):
        if (msg.bottom):
            rospy.wait_for_service('stop', timeout=3)
            stop_action = rospy.ServiceProxy('stop', Empty)
            stop_action()

    self.light_cb(self, msg):
        if (msg.data < self.light_threshold):
            rospy.wait_for_service('stop', timeout=3)
            close_action = rospy.ServiceProxy('stop', Empty)
            close_action() 
        pass

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

        if self.cur_state != self.past_state:

            elif self.cur_state == STATE_PICKING:
                try:
                    self.target_pose = rospy.wait_for_message('/obj_pose', PoseStamped, timeout=3)
                    self.pub_target_pose.publish(self.target_pose)
                    plan = self.move_group.plan()
                    if plan.joint_trajectory.header.frame_id == "":    # No planning result
                        self.next_state = STATE_PLAN_ERROR
                        return
                    else:
                        rospy.sleep(2)
                        self.move_group.go(wait=True)
                        if self.holding_obj == "large_cuboid":
                            self.cnt_large_cuboid += 1
                        elif self.holding_obj == "medium_cuboid":
                            self.cnt_medium_cuboid += 1
                        elif self.holding_obj == "cylinder":
                            self.cnt_cylinder += 1
                            self.holding_obj = None

                        rospy.wait_for_service('/robotiq_finger_control_node/close_gripper', timeout=3)
                        close_action = rospy.ServiceProxy('/robotiq_finger_control_node/close_gripper', Empty)
                        close_action()
                        rospy.sleep(1)
                except rospy.ROSException as e:
                    rospy.logerr('No target pose to conduct picking task.', e)
                    self.next_state = STATE_PLAN_ERROR
                    return
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
        node.process()
        r.sleep()
    rospy.spin()
