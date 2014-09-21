#include "ros/ros.h"
#include "transport_priority/content.h"

#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher high_pub, normal_pub, low_pub;

  high_pub = n.advertiseWithQoS<transport_priority::content>("topic_high", 1000, ros::High);
  normal_pub = n.advertiseWithQoS<transport_priority::content>("topic_normal", 1000, ros::Normal);
  low_pub = n.advertiseWithQoS<transport_priority::content>("topic_low", 1000, ros::Low);

  ros::Rate loop_rate(20);

  int count = 0;
  while (ros::ok())
  {
    transport_priority::content msg;

    msg.id=count;
    for (int i=0;i<5000000; i++)
        msg.data.push_back('a');

    ROS_INFO("Publishing message with id [%d]", msg.id);
    low_pub.publish(msg);
    normal_pub.publish(msg);
    high_pub.publish(msg);



    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }


  return 0;
}
