#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Duration d(2, 0);
  ros::SubscribeQoSOptions qos_ops;
  qos_ops.time_filter_duration=d;
  ros::Subscriber sub = n.subscribeWithQoS("chatter", 1000, qos_ops, chatterCallback);
  ros::spin();

  return 0;
}
