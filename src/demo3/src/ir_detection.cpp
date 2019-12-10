#include "ros/ros.h"
#include <wiringPi.h>
#include <iostream>
#include <std_msgs/Int16.h>
#include "mobilerobot_msgs/Collision.h"

using namespace std;

const short int PIN_IR_RECEIVER = 7;


ros::Time last_r_time;
ros::Time last_l_time;
ros::Time last_b_time;

mobilerobot_msgs::Collision msg;

void ir_cb(void) {
    ros::Time diff_time = ros::Time::now() - last_r_time;
    if(diff_time > ros::Duration(0.001)){
        // msg.right = true;
        last_r_time = ros::Time::now();
        if(diff_time < ros::Duration(0.0015))
            cout << "beacon-1" << endl;
        else if(diff_time < ros::Duration(0.003))
            cout << "beacon-2" << endl;
    }        
}

int main (int argc, char **argv){
    ros::init(argc, argv, "collision_detection_node");
    ros::NodeHandle nh;
    ros::Publisher pub_collision = nh.advertise<mobilerobot_msgs::Collision>("collision_data", 1);
    last_ir_time = ros::Time::now();
    msg.header.frame_id = "base_link";
    
    // WiringPi library setup
    setenv("WIRINGPI_GPIOMEM", "1", 1);
    wiringPiSetup();
    pinMode(PIN_IR_RECEIVER, INPUT);

    // Interrupt setup: INT_EDGE_FALLING, INT_EDGE_RISING
    if(wiringPiISR (PIN_IR_RECEIVER, INT_EDGE_RISING, &ir_cb) < 0){
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        return 1;
    }

    // 2 Hz
    ros::Rate loop_rate(2);
    while(ros::ok()) {
        // if(digitalRead(PIN_TOUCH_BOTTOM) == 0)
        //     msg.bottom = false;
        // if(digitalRead(PIN_TOUCH_RIGHT) == 0)
        //     msg.right = false;
        // if(digitalRead(PIN_TOUCH_LEFT) == 0)
        //     msg.left = false;

        // msg.header.stamp = ros::Time::now();
        // pub_collision.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0 ;
}
