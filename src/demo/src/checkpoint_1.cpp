#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>

void number_callback(const std_msgs::Int32::ConstPtr& msg)
{
	if(msg->data != 0)
	{
		printf("message from Arduino is %d\n", msg->data);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "checkpoint_1");
	ros::NodeHandle nh;
	ros::Subscriber number_subscriber = nh.subscribe("/number_arduino", 10, number_callback);
	ros::Publisher number_publisher = nh.advertise<std_msgs::Int32>("/number", 10);
	ros::Rate loop_rate(10);
	
	ros::Duration(2).sleep();
	std_msgs::Int32 msg;
	msg.data = 0;
	number_publisher.publish(msg);
	ros::topic::waitForMessage<std_msgs::Int32>("/number_arduino", ros::Duration(5));

	
	
	int number_count = 0;
	while(ros::ok() && number_count < 1000)
	{
		std_msgs::Int32 msg;
		msg.data = number_count++;
		number_publisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		if(msg.data != 0)
		{
			printf("user's input is %d\n", msg.data);
		}
	}
	return 0;
}
