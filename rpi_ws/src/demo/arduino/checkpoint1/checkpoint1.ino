#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;
std_msgs::Int32 msg;
ros::Publisher number_pub("/number_arduino", &msg);

void number_cb(const std_msgs::Int32& number_msg)
{
  msg.data = number_msg.data;
  msg.data = msg.data * 2;
  number_pub.publish(&msg);
}

ros::Subscriber<std_msgs::Int32> number_sub("/number", &number_cb);

void setup()
{
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(number_sub);
  nh.advertise(number_pub);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
