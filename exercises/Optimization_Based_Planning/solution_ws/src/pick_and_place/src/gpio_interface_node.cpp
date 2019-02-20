#include <ros/ros.h>


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_gpio");

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);



  //exit
  return 0;
}
