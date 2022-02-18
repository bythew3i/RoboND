#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
const float pick_up_pose[3] = {5, 7, 1}; // x, y, w
const float drop_off_pose[3] = {6, -7, 1}; // x, y, w

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a pick up position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pick_up_pose[0];
  goal.target_pose.pose.position.y = pick_up_pose[1];
  goal.target_pose.pose.orientation.w = pick_up_pose[2];

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Robot is traveling to the pick up zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    ROS_INFO("Robot picked up the virtual object");
  else{
    ROS_ERROR("Robot failed to move to the pick up zone");
    exit(0);
  }
    
  ros::Duration(5.0).sleep();

  // Define a drop off position and orientation for the robot to reach
  goal.target_pose.pose.position.x = drop_off_pose[0];
  goal.target_pose.pose.position.y = drop_off_pose[1];
  goal.target_pose.pose.orientation.w = drop_off_pose[2];

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Robot is traveling to the drop off zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    ROS_INFO("Robot dropped off the virtual object");
  else {
    ROS_ERROR("Robot failed to move to the drop off zone");
    exit(0);
  }

  while (ros::ok()) {
    sleep(1);
  }
}