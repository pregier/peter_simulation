#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle nh("~");

  double position_x, position_y, yaw;

  if (!nh.getParam("position_x", position_x))
  {
    ROS_WARN("The parameter position_x is not set. Please check your launch file");
    return 0;
  }
  else
  {
    ROS_INFO("position_x is set to %f", position_x);
  }

  if (!nh.getParam("position_y", position_y))
  {
    ROS_WARN("The parameter position_y is not set. Please check your launch file");
    return 0;
  }
  else
  {
    ROS_INFO("position_y is set to %f", position_y);
  }

  if (!nh.getParam("yaw", yaw))
  {
    ROS_WARN("The parameter orientation is not set. Please check your launch file");
    return 0;
  }
  else
  {
    ROS_INFO("yaw is set to %f", yaw);
  }

  tf::Quaternion yaw_q;
  yaw_q.setEuler(0.0, 0.0, yaw);
  //tell the action client that we want to spin a thread by default
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = position_x;
  goal.target_pose.pose.position.y = position_y;
  goal.target_pose.pose.position.z = 0.0;

  goal.target_pose.pose.orientation.w = yaw_q.getW();
  goal.target_pose.pose.orientation.x = yaw_q.getX();
  goal.target_pose.pose.orientation.y = yaw_q.getY();
  goal.target_pose.pose.orientation.z = yaw_q.getZ();

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("!!!!!!!!!!Hooray, the base reached the goal"); // moved 1 meter forward
  else
    ROS_INFO("The base failed to move to the goal: %s", ac.getState().getText().c_str());  // move forward 1 meter for some reason

  return 0;
}

