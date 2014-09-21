#include "ros/ros.h"
#include "transport_priority/content.h"

void TopicHighCallback(const transport_priority::content::ConstPtr& msg)
{
  ROS_INFO("I heard message [%d] on topic_high", msg->id);
}

void TopicNormalCallback(const transport_priority::content::ConstPtr& msg)
{
  ROS_INFO("I heard message [%d] on topic_normal", msg->id);
}

void TopicLowCallback(const transport_priority::content::ConstPtr& msg)
{
  ROS_INFO("I heard message [%d] on topic_low", msg->id);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub_high = n.subscribe("topic_high", 1000, TopicHighCallback);
  ros::Subscriber sub_normal = n.subscribe("topic_normal", 1000, TopicNormalCallback);
  ros::Subscriber sub_low = n.subscribe("topic_low", 1000, TopicLowCallback);
  ros::spin();

  return 0;
}
