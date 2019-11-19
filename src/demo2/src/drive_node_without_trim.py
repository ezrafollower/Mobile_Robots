#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

def main():
    rospy.init_node('drive_node')

    pub_r = rospy.Publisher("right_pwm", Int16, queue_size = 10)
    pub_l = rospy.Publisher("left_pwm",  Int16, queue_size = 10)

    while not rospy.is_shutdown():
        try:
            pwm_r = int(raw_input("user's right:"))
        except ValueError:
            rospy.logerr("Invalid input, set default value 0")
            pwm_r = 0
        try:
            pwm_l = int(raw_input("user's left:"))
        except ValueError:
            rospy.logerr("Invalid input, set default value 0")
            pwm_l = 0
        
        # Go forward input_r=input_l=111
        if (pwm_r == pwm_l):
            trim = 0 #-11
            pwml_raw = pwm_l
            pwmr_raw = pwm_r
            if(pwm_r != 0 ):
                pwm_r = pwmr_raw + trim
            if(pwm_l != 0 ):
                pwm_l = pwml_raw - trim
            print pwm_l, pwm_r
            pub_r.publish(Int16(pwm_r))
            pub_l.publish(Int16(pwm_l))
            rospy.sleep(1.5) #6.5
	
        # Other motion
	else:
            trim = 0 #-5
            pwml_raw = pwm_l
            pwmr_raw = pwm_r
            if(pwm_r != 0 ):
                pwm_r = pwmr_raw + trim
            if(pwm_l != 0 ):
                pwm_l = pwml_raw - trim
            print pwm_l, pwm_r
            pub_r.publish(Int16(pwm_r))
            pub_l.publish(Int16(pwm_l))
            rospy.sleep(1.5) #4.0

        pub_r.publish(Int16(0))
        pub_l.publish(Int16(0))


if __name__ == "__main__":
    main()
