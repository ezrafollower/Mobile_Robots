#include "ros/ros.h"
#include <wiringPi.h>
#include <iostream>
#include <std_msgs/Int16.h>
#include "mobilerobot_msgs/BeaconInfo.h"

using namespace std;

const short int PIN_IR_RECEIVER = 7;


ros::Time last_ir_time;

mobilerobot_msgs::BeaconInfo msg;

volatile uint16_t cnt_beacon1500 = 0;
volatile uint16_t cnt_beacon600 = 0;
bool flag_busy = false;

void ir_cb(void) {
    if(flag_busy == false){
        ros::Duration diff_time = ros::Time::now() - last_ir_time;
        if(diff_time > ros::Duration(0.001)){
            // msg.right = true;
            ros::Duration diff_time = ros::Time::now() - last_ir_time;
            if(diff_time > ros::Duration(0.0007) && diff_time < ros::Duration(0.002))
                // cout << diff_time << "beacon-1 600" << endl;
                cnt_beacon600++;
            else if(diff_time > ros::Duration(0.002) && diff_time < ros::Duration(0.004))
                // cout << diff_time << "beacon-2 1500" << endl;
                cnt_beacon1500++;
        }
        last_ir_time = ros::Time::now();
    }   
}

int main (int argc, char **argv){
    ros::init(argc, argv, "beacon_detection_node");
    ros::NodeHandle nh;
    ros::Publisher pub_beacon = nh.advertise<mobilerobot_msgs::BeaconInfo>("beacon_data", 1);
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

    // 4 Hz
    ros::Rate loop_rate(4);
    while(ros::ok()) {
        flag_busy = true;
        
        if(cnt_beacon600 - cnt_beacon1500 >= 10){
            msg.beacon600 = true;
        }
        else if(cnt_beacon1500 - cnt_beacon600 >= 10){
            msg.cnt_beacon1500 = true;
        }
        else{
            msg.cnt_beacon600 = false;
            msg.cnt_beacon1500 = false;
        }

        msg.header.stamp = ros::Time::now();
        pub_beacon.publish(msg);

        cnt_beacon600 = cnt_beacon1500 = 0;
        flag_busy = false;
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0 ;
}
