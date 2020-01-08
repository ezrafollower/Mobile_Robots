#include "ros/ros.h"
#include <wiringPi.h>
#include <iostream>
#include <std_msgs/Int16.h>
#include "mobilerobot_msgs/Collision.h"

using namespace std;

const short int PIN_TOUCH_RIGHT = 0;
const short int PIN_TOUCH_LEFT = 2;
const short int PIN_TOUCH_BOTTOM = 3;

ros::Time last_r_time;
ros::Time last_l_time;
ros::Time last_b_time;

mobilerobot_msgs::Collision msg;

void touch_right_cb(void) {
    if(ros::Time::now() - last_r_time > ros::Duration(0.3) && digitalRead(PIN_TOUCH_RIGHT) == 1){
        msg.right = true;
        last_r_time = ros::Time::now();
    }        
}
void touch_left_cb(void) {
    if(ros::Time::now() - last_l_time > ros::Duration(0.3) && digitalRead(PIN_TOUCH_LEFT) == 1){
        msg.left = true;
        last_l_time = ros::Time::now();
    } 
}
void touch_bottom_cb(void) {
    if(ros::Time::now() - last_b_time > ros::Duration(0.3) && digitalRead(PIN_TOUCH_BOTTOM) == 1){
        msg.bottom = true;
        last_b_time = ros::Time::now();
    } 
}

int main (int argc, char **argv){
    ros::init(argc, argv, "collision_detection_node");
    ros::NodeHandle nh;
    ros::Publisher pub_collision = nh.advertise<mobilerobot_msgs::Collision>("collision_data", 1);
    last_r_time = ros::Time::now();
    last_l_time = ros::Time::now();
    last_b_time = ros::Time::now();
    msg.header.frame_id = "base_link";
    
    // WiringPi library setup
    setenv("WIRINGPI_GPIOMEM", "1", 1);
    wiringPiSetup();
    pinMode(PIN_TOUCH_RIGHT, INPUT);
    pinMode(PIN_TOUCH_LEFT, INPUT);
    pinMode(PIN_TOUCH_BOTTOM, INPUT);

    // Interrupt setup: INT_EDGE_FALLING, INT_EDGE_RISING
    if(wiringPiISR (PIN_TOUCH_RIGHT, INT_EDGE_RISING, &touch_right_cb) < 0){
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        return 1;
    }
    if(wiringPiISR (PIN_TOUCH_LEFT, INT_EDGE_RISING, &touch_left_cb) < 0){
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        return 1;
    }
    if(wiringPiISR (PIN_TOUCH_BOTTOM, INT_EDGE_RISING, &touch_bottom_cb) < 0){
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        return 1;
    }

    // 2 Hz
    ros::Rate loop_rate(2);
    while(ros::ok()) {
        if(digitalRead(PIN_TOUCH_BOTTOM) == 0)
            msg.bottom = false;
        if(digitalRead(PIN_TOUCH_RIGHT) == 0)
            msg.right = false;
        if(digitalRead(PIN_TOUCH_LEFT) == 0)
            msg.left = false;

        msg.header.stamp = ros::Time::now();
        pub_collision.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0 ;
}
