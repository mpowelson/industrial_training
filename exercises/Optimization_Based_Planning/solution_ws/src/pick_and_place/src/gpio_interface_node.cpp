#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_gpio");
  ros::NodeHandle nh;

  ros::Publisher modbus_pub = nh.advertise<std_msgs::Int32MultiArray>("modbus_wrapper/output", 1000);

  std_msgs::Int32MultiArray msg;
  while( ros::ok())
  {
    // Set all IO to active (high) 0b111111
    std::vector<int> all_active = {63};
    msg.data = all_active;
    modbus_pub.publish(msg);
    ros::Duration(1.0).sleep();

    // Set all IO to inactive (low) 0b000000
    std::vector<int> all_inactive = {0};
    msg.data = all_inactive;
    modbus_pub.publish(msg);
    ros::Duration(1.0).sleep();

    // Set some IO active - some inactive 0b000111
    std::vector<int> half_active = {7};
    msg.data = half_active;
    modbus_pub.publish(msg);
    ros::Duration(1.0).sleep();
  }

  //exit
  return 0;
}
