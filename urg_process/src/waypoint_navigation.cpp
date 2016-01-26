#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient ac;
move_base_msgs::MoveBaseGoal goal;
float[] recorded_waypoints;

int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_navigation");

  //tell the action client that we want to spin a thread by default
  ac = MoveBaseClient("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  ros::Subscriber sub = n.subscribe("/amcl_pose", 1000, amclCallback);
  ros::spin();

  return 0;
}

int loadWaypoints(){
  std::ifstream ifs("waypoint.txt");
  std::string str;
  ifs>>str;
  std::cout<<str<<std::endl;

  FILE *fp;
  fopen(&fp,"waypoint.txt","r");
}

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
}