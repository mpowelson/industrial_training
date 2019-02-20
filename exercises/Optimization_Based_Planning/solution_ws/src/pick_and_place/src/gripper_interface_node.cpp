#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa325_wsg50/WSG50HomingAction.h>
#include <ipa325_wsg50/WSG50GraspPartAction.h>
#include <ipa325_wsg50/WSG50ReleasePartAction.h>
#include <ipa325_wsg50/ackFastStop.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_gripper");
  ros::NodeHandle nh;

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ipa325_wsg50::WSG50HomingAction> home_ac("WSG50Gripper_Homing", true);
  actionlib::SimpleActionClient<ipa325_wsg50::WSG50GraspPartAction> grasp_ac("WSG50Gripper_GraspPartAction", true);
  actionlib::SimpleActionClient<ipa325_wsg50::WSG50ReleasePartAction> release_ac("WSG50Gripper_ReleasePartAction", true);
  ros::ServiceClient srv = nh.serviceClient<ipa325_wsg50::ackFastStop>("/AcknowledgeFastStop");

  ROS_INFO("Waiting for action servers to start.");
  home_ac.waitForServer(); //will wait for infinite time
  grasp_ac.waitForServer();
  release_ac.waitForServer();
  ipa325_wsg50::ackFastStop ack;
  srv.call(ack);

  ROS_INFO("Homing gripper...");
  ipa325_wsg50::WSG50HomingGoal home_goal;
  home_goal.direction = true;   // True is in the out direction
  home_ac.sendGoal(home_goal);
  bool finished_before_timeout = home_ac.waitForResult(ros::Duration(10.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = home_ac.getState();
    ROS_INFO("Homing finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Homing did not finish before the time out.");


  ROS_INFO("Grasping Part...");
  ipa325_wsg50::WSG50GraspPartGoal grasp_goal;
  grasp_goal.width = 80;  // Part width in mm
  grasp_goal.speed = 20;  // Speed in mm/s
  grasp_ac.sendGoal(grasp_goal);
  finished_before_timeout = grasp_ac.waitForResult(ros::Duration(15.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = grasp_ac.getState();
    ROS_INFO("Grasp finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Grasp did not finish before the time out.");


  ROS_INFO("Releasing Part...");
  ipa325_wsg50::WSG50ReleasePartGoal release_goal;
  release_goal.openwidth = 110;
  release_goal.speed = 50;
  release_ac.sendGoal(release_goal);
  finished_before_timeout = release_ac.waitForResult(ros::Duration(10.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = release_ac.getState();
    ROS_INFO("Release finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Release did not finish before the time out.");


  //exit
  return 0;
}
