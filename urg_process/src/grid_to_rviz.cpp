#include "grid_to_rviz.h"


void chatterCallback(const nav_msgs::GridCells::ConstPtr& msg){
  
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "laser_grid";
  marker.id = 0;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.lifetime = ros::Duration();

  int msgNum = msg->cells.size();
  for(int i = 0;i<msgNum;i++){
     marker.points.push_back(msg->cells[i]);
  }

  marker_pub.publish(marker);

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("laser_to_grid", 1000, chatterCallback);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  
  ros::spin();
}
// %EndTag(FULLTEXT)%

