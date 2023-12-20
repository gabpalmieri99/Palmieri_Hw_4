#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 
int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
 
  move_base_msgs::MoveBaseGoal goal1, goal2, goal3, goal4;
  //we'll send a goal to the robot to move 1 meter forward
  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.header.stamp = ros::Time::now();
  goal1.target_pose.pose.position.x = -10.0;
  goal1.target_pose.pose.position.y = 3.0;
  goal1.target_pose.pose.orientation.w = 1.0;


  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.header.stamp = ros::Time::now();
  goal2.target_pose.pose.position.x = -15.0;
  goal2.target_pose.pose.position.y = 7.0;
  goal2.target_pose.pose.orientation.w = 1.0;

  goal3.target_pose.header.frame_id = "map";
  goal3.target_pose.header.stamp = ros::Time::now();
  goal3.target_pose.pose.position.x = -6.0;
  goal3.target_pose.pose.position.y = 8.0;
  goal3.target_pose.pose.orientation.w = 1.0;

  goal4.target_pose.header.frame_id = "map";
  goal4.target_pose.header.stamp = ros::Time::now();
  goal4.target_pose.pose.position.x = -17.5;
  goal4.target_pose.pose.position.y = 3.0;
  goal4.target_pose.pose.orientation.w = 1.0;
 
  ROS_INFO("Sending goal 3");
  ac.sendGoal(goal1);
  ac.waitForResult();
  
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved til goal 3");
    else
      ROS_INFO("The base failed to move for some reason");

  if (ac.waitForResult()){
    ROS_INFO("Sending goal 4");
    ac.sendGoal(goal2);
    ac.waitForResult();
  } 

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved til goal 4");
    else
      ROS_INFO("The base failed to move for some reason");

  if (ac.waitForResult()){
    ROS_INFO("Sending goal 2");
    ac.sendGoal(goal3);
    ac.waitForResult();
  } 
  
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved til goal 2");
    else
      ROS_INFO("The base failed to move for some reason");

  if (ac.waitForResult()){
    ROS_INFO("Sending goal 1");
    ac.sendGoal(goal4);
    ac.waitForResult();
  } 
   
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved til goal 1");
    else
      ROS_INFO("The base failed to move for some reason");
 
   return 0;
 }