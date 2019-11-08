#include "ros/ros.h"
#include <wiringPi.h>
#include <iostream>
#include <std_msgs/Int16.h>
//light receive pin 3
const short int lightpin = 3;
ros::Time previous_time;
ros::Time current_time;

int main (int argc, char **argv){
    ros::init(argc, argv, "light_receive_data");
    ros::NodeHandle n;
    ros::Publisher light_pub = n.advertise<std_msgs::Int16>("light_data", 1);
    unsigned short int light_rev = 0;

    std_msgs::Int16 light_data;
    //use this command whithout sudo
    setenv("WIRINGPI_GPIOMEM", "1", 1);
    //library setup function
    wiringPiSetup () ;
    pinMode (lightpin, INPUT) ;
    //10hz
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        light_rev = digitalRead(lightpin) ;
        light_data.data = light_rev;
         ROS_INFO("light_receive : %d ", light_rev);
        light_pub.publish(light_data);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0 ;
}