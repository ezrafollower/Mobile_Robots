#!/usr/bin/env python2.7
from math import pi, sqrt
import sys
import copy
import random
import copy

import rospy

# FSM ID
STATE_UNKOWN = -1
STATE_IDLE = 0
STATE_RECOGNITION = 1
STATE_POSE_ESTIMATION = 2
STATE_PICKING = 3
STATE_APRILTAG_LOC = 4
STATE_PLACING = 5
STATE_GO_HOME = 6
STATE_IMAGE_COLLECTION = 7
STATE_PLAN_ERROR = 8
STATE_RANDOM_PICKING = 10


class FSM(object):
    """docstring for FSM"""
    def __init__(self):
        self.next_state = STATE_UNKOWN
        self.cur_state = STATE_UNKOWN
        self.past_state = STATE_UNKOWN

        # ROS publisher & subscriber & service
        self.pub_target_pose = rospy.Publisher('target_pose', PoseStamped, queue_size=1)
        self.home_srv = rospy.Service("start", Trigger, self.home_cb)
        self.image_collect_srv = rospy.Service("stop", Trigger, self.image_collect_cb)

    ############################ FSM Transition callback ############################
    def calibrate_cb(self, req):
        self.next_state = STATE_CALIBRATION
        if self.cur_state != self.next_state and self.cur_state != STATE_PLAN_ERROR:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            self.next_state = self.cur_state
            success, message = (False, "Request denied")

    def loc_cb(self, req):
        self.next_state = STATE_APRILTAG_LOC
        if self.cur_state != self.next_state and self.cur_state != STATE_PLAN_ERROR:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            self.next_state = self.cur_state
            return TriggerResponse(success=False, message="Request denied")

    def image_collect_cb(self, req):
        self.next_state = STATE_IMAGE_COLLECTION
        if self.cur_state != self.next_state and self.cur_state != STATE_PLAN_ERROR:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            self.next_state = self.cur_state
            return TriggerResponse(success=False, message="Request denied")

    def home_cb(self, req):
        self.next_state = STATE_GO_HOME
        if self.cur_state != self.next_state:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            return TriggerResponse(success=False, message="Request denied")

    def pick_cb(self, req):
        self.next_state = STATE_PICKING
        if self.cur_state != self.next_state and self.cur_state != STATE_PLAN_ERROR:
            return TriggerResponse(success=True, message="Request accepted.")
        else:
            self.next_state = self.cur_state
            return TriggerResponse(success=False, message="Request denied")


    ############################ FSM State main task ############################
    def process(self):
        # Update FSM state
        self.past_state = self.cur_state
        self.cur_state = self.next_state

        if self.cur_state != self.past_state:
            if self.cur_state == STATE_RECOGNITION:
                pass
            elif self.cur_state == STATE_POSE_ESTIMATION:
                pass

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

            elif self.cur_state == STATE_RANDOM_PICKING:
                while not rospy.is_shutdown():
                    p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
                    p.pose.position.x = random.uniform(-0.5, -0.3)
                    p.pose.position.y = random.uniform(-0.25, 0.25)
                    p.pose.position.z = 0.28
                    q = tf.transformations.quaternion_from_euler(pi, pi/2, random.uniform(-pi/3, pi/3))
                    p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                    self.move_group.set_pose_target(p.pose)
                    self.pub_target_pose.publish(p)
                    plan = self.move_group.plan()
                    if plan.joint_trajectory.header.frame_id == "":    # No planning result
                        self.next_state = STATE_PLAN_ERROR
                        return
                    else:
                        rospy.sleep(1)
                        self.move_group.go(wait=True)


            elif self.cur_state == STATE_APRILTAG_LOC:
                # Fake holding obj
                self.holding_obj = random.choice(["large_cuboid", "medium_cuboid", "cylinder"])

                # x, y, z = [-0.4, 0.2, 0.5]
                self.move_group.clear_pose_targets()
                self.move_group.set_joint_value_target(JOINTS_PLACING_HOME)
                plan = self.move_group.plan()
                self.move_group.go(wait=True)
                rospy.sleep(1)
                
                # Apriltag detection
                p = None
                msg = None
                try:
                    msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray, timeout=3)
                    flag_found_tag = False
                    for i in range(len(msg.detections)):
                        if msg.detections[i].id == PLACING_TAG_ID:
                            # Calculate the position where the object to place
                            tmp_p = copy.deepcopy(msg.detections[i].pose)
                            flag_found_tag = True
                            if self.holding_obj == "large_cuboid":
                                tmp_p.pose.position.x = tmp_p.pose.position.x + LARGE_CUBOID_PLACING_OFFSET[0]
                                tmp_p.pose.position.y = tmp_p.pose.position.y + \
                                                        LARGE_CUBOID_PLACING_OFFSET[1] + 0.07 * self.cnt_large_cuboid
                                rospy.loginfo("place to large_cuboid: {}, {}".format(tmp_p.pose.position.x, tmp_p.pose.position.y))
                                p = self.tf_listener.transformPose('base_link', tmp_p)
                            elif self.holding_obj == "medium_cuboid":
                                tmp_p.pose.position.x = tmp_p.pose.position.x + MEDIUM_CUBOID_PLACING_OFFSET[0]
                                tmp_p.pose.position.y = tmp_p.pose.position.y + \
                                                        MEDIUM_CUBOID_PLACING_OFFSET[1] + 0.07 * self.cnt_medium_cuboid
                                rospy.loginfo("place to medium_cuboid: {}, {}".format(tmp_p.pose.position.x, tmp_p.pose.position.y))
                                p = self.tf_listener.transformPose('base_link', tmp_p)
                            elif self.holding_obj == "cylinder":
                                tmp_p.pose.position.x = tmp_p.pose.position.x + CYLINDER_PLACING_OFFSET[0]
                                tmp_p.pose.position.y = tmp_p.pose.position.y + \
                                                        CYLINDER_PLACING_OFFSET[1] + 0.07 * self.cnt_cylinder
                                rospy.loginfo("place to cylinder: {}, {}".format(tmp_p.pose.position.x, tmp_p.pose.position.y))
                                p = self.tf_listener.transformPose('base_link', tmp_p)
                            else:
                                rospy.logerr('Object picking failed')
                                self.next_state = STATE_PLAN_ERROR
                                return
                    if not flag_found_tag:
                        rospy.logerr('Could not find tag to conduct placing task')
                        self.next_state = STATE_PLAN_ERROR
                        return

                except rospy.exceptions.ROSException as e:
                    rospy.logerr(e)
                    self.next_state = STATE_PLAN_ERROR
                    return

                p.pose.position.z = FIXED_PLACING_HEIGHT
                euler = tf.transformations.euler_from_quaternion([p.pose.orientation.x,
                                                                    p.pose.orientation.y,
                                                                    p.pose.orientation.z,
                                                                    p.pose.orientation.w])
                if pi - euler[2] + pi/2 > pi*3/2:
                    rotation = -euler[2] + pi/2 
                elif pi - euler[2] + pi/2 < pi/2:
                    rotation = pi*2 - euler[2] + pi/2
                else:
                    rotation = pi - euler[2] + pi/2
                q = tf.transformations.quaternion_from_euler(pi, pi/2, 0)
                p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                self.target_pose = copy.deepcopy(p)
                self.pub_target_pose.publish(p)
                self.next_state = STATE_PLACING
                rospy.loginfo('Ready to place')

            elif self.cur_state == STATE_PLACING:
                if self.target_pose != None:
                    self.move_group.set_pose_target(self.target_pose.pose)
                    self.pub_target_pose.publish(self.target_pose)
                    plan = self.move_group.plan()
                    if plan.joint_trajectory.header.frame_id == "":    # No planning result
                        self.next_state = STATE_PLAN_ERROR
                        return
                    else:
                        rospy.sleep(2)
                        self.move_group.go(wait=True)
                    # Release holding object
                    if self.holding_obj == "large_cuboid":
                        self.cnt_large_cuboid += 1
                    elif self.holding_obj == "medium_cuboid":
                        self.cnt_medium_cuboid += 1
                    elif self.holding_obj == "cylinder":
                        self.cnt_cylinder += 1
                    self.holding_obj = None

                    rospy.wait_for_service('/robotiq_finger_control_node/open_gripper', timeout=3)
                    open_action = rospy.ServiceProxy('/robotiq_finger_control_node/open_gripper', Empty)
                    open_action()
                    rospy.sleep(1)

                    self.next_state = STATE_GO_HOME
                else:
                    rospy.logerr('No target pose to placing')
                    self.next_state = STATE_PLAN_ERROR
                    return


            elif self.cur_state == STATE_PLAN_ERROR:
                rospy.logerr('PLANNING ERROR STATE')

            elif self.cur_state == STATE_GO_HOME:
                self.move_group.clear_pose_targets()
                self.move_group.set_joint_value_target(JOINTS_PICKING_HOME)
                plan = self.move_group.plan()
                self.move_group.go(wait=True)
                rospy.loginfo('Ready to pick')

            elif self.cur_state == STATE_IMAGE_COLLECTION:
                # center
                p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
                p.pose.position.x = -0.5
                p.pose.position.y = 0.0
                p.pose.position.z = 0.4
                q = tf.transformations.quaternion_from_euler(pi, pi/2, 0)
                p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                self.move_group.set_pose_target(p.pose)
                self.pub_target_pose.publish(p)
                plan = self.move_group.plan()
                rospy.sleep(2)
                self.move_group.go(wait=True)

                # right
                p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
                p.pose.position.x = -0.5
                p.pose.position.y = 0.4/sqrt(2)
                p.pose.position.z = 0.4/sqrt(2)
                q = tf.transformations.quaternion_from_euler(pi/2, pi/4, -pi/2)
                p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                self.move_group.set_pose_target(p.pose)
                self.pub_target_pose.publish(p)
                plan = self.move_group.plan()
                rospy.sleep(2)
                self.move_group.go(wait=True)

                # center
                p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
                p.pose.position.x = -0.5
                p.pose.position.y = 0.0
                p.pose.position.z = 0.4
                q = tf.transformations.quaternion_from_euler(pi, pi/2, 0)
                p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                self.move_group.set_pose_target(p.pose)
                self.pub_target_pose.publish(p)
                plan = self.move_group.plan()
                self.move_group.go(wait=True)

                # front
                p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
                p.pose.position.x = -0.5 + 0.4/sqrt(2)
                p.pose.position.y = 0.0
                p.pose.position.z = 0.4/sqrt(2)
                q = tf.transformations.quaternion_from_euler(0, pi/4, pi)
                p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                self.move_group.set_pose_target(p.pose)
                self.pub_target_pose.publish(p)
                plan = self.move_group.plan()
                rospy.sleep(2)
                self.move_group.go(wait=True)

                # center
                p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
                p.pose.position.x = -0.5
                p.pose.position.y = 0.0
                p.pose.position.z = 0.4
                q = tf.transformations.quaternion_from_euler(pi, pi/2, 0)
                p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                self.move_group.set_pose_target(p.pose)
                self.pub_target_pose.publish(p)
                plan = self.move_group.plan()
                self.move_group.go(wait=True)

                # left
                p = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.robot.get_planning_frame()))
                p.pose.position.x = -0.5
                p.pose.position.y = -0.4/sqrt(2)
                p.pose.position.z = 0.4/sqrt(2)
                q = tf.transformations.quaternion_from_euler(-pi/2, pi/4, pi/2)
                p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                self.move_group.set_pose_target(p.pose)
                self.pub_target_pose.publish(p)
                plan = self.move_group.plan()
                rospy.sleep(2)
                self.move_group.go(wait=True)
            
    def update_collision_object(self):
        self.scene.remove_world_object('table')
        obj_name = "table"
        obj_pose = PoseStamped()
        obj_pose.header.frame_id = self.robot.get_planning_frame()
        obj_pose.pose.orientation.w = 1.0
        obj_pose.pose.position.z = -0.01
        self.scene.add_box(obj_name, obj_pose, size=(1.5, 1.5, 0.001))
        self.scene.remove_world_object('back_wall')


    def shutdown_cb(self):
        rospy.loginfo("Node shutdown")


if __name__ == '__main__':
    rospy.init_node('final_round_node', anonymous=False)
    node = FSM()
    rospy.on_shutdown(node.shutdown_cb)

    while not rospy.is_shutdown():
        node.process()
        rospy.sleep(0.5)
    rospy.spin()
