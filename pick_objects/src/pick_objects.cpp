#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");
  //ros::Rate loop_rate(10);
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //setup node to broadcast robot reached target
  ros::Publisher target_pub = n.advertise<std_msgs::UInt8>("/at_target", 1);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(10.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal1;
  move_base_msgs::MoveBaseGoal goal2;

  // set up the frame parameters
  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal1.target_pose.pose.position.x = -1.0;
  goal1.target_pose.pose.position.y = 3.0;
  goal1.target_pose.pose.orientation.w = 1.0;

  sleep(5);

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal 1");
  ac.sendGoal(goal1);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Robot reached pickup zone");
    std_msgs::UInt8 msg;
    msg.data = 2;
    //wait for 5 seconds
    ROS_INFO("Publish message %d", msg.data);
    target_pub.publish(msg);
    ros::spinOnce();
    sleep(5);
  }
  else
    ROS_INFO("The base failed to move to pickup zone.");

  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.header.stamp = ros::Time::now();
  // goal2.target_pose.pose.position.x = 0.0;
  // goal2.target_pose.pose.position.y = 0.0;
  goal2.target_pose.pose.position.x = 2.0;
  goal2.target_pose.pose.position.y = -1.0;
  goal2.target_pose.pose.orientation.w = 1.0;

       // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal 2");
  ac.sendGoal(goal2);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std_msgs::UInt8 msg2;
    ROS_INFO("Robot reached drop off zone");
    msg2.data = 4;
    ROS_INFO("Publish message %d", msg2.data);
    target_pub.publish(msg2);
    ros::spinOnce();
  }
  else
    ROS_INFO("The base failed to move to goal 2.");

  ros::spin();
}