#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <urg_process/VectorField.h>

ros::Publisher pubTrackMarkers;
const float CUBE_LENGTH = 0.025f;


void pointCallback(const urg_process::VectorField::ConstPtr& msg){

  visualization_msgs::Marker markerMsg;
  markerMsg.header.frame_id  = msg->child_frame_id;
  markerMsg.ns = "point_cloud";
  markerMsg.action = visualization_msgs::Marker::ADD;
  markerMsg.pose.orientation.w = 1.0;
  markerMsg.type = visualization_msgs::Marker::CUBE_LIST;
  markerMsg.scale.x = CUBE_LENGTH;
  markerMsg.scale.y = CUBE_LENGTH;
  markerMsg.scale.z = CUBE_LENGTH;

  std_msgs::ColorRGBA c;
  c.r = 0;
  c.g = 1;
  c.b = 0;
  c.a = 1;

  for(int i=0; i<msg->data_num; i++){
    geometry_msgs::Point tmp_p;
    tmp_p.x = msg->pos[i].x; tmp_p.y = msg->pos[i].y; tmp_p.z = msg->pos[i].z;
    markerMsg.points.push_back(tmp_p);
    markerMsg.colors.push_back(c);
  }

  pubTrackMarkers.publish(markerMsg);
  markerMsg.points.clear();
  markerMsg.colors.clear();

  int j = msg->data_num;
  ROS_INFO("publish point %d",j);

}

int main( int argc, char** argv ){

  ros::init(argc, argv, "visualize_vector_field");
  ros::NodeHandle n;

  ros::Rate r(30);

  pubTrackMarkers = n.advertise<visualization_msgs::Marker>("/point_cloud",1);

  ros::Subscriber sub = n.subscribe("height_point", 1000, pointCallback);
  ros::spin();

  return 0;
}