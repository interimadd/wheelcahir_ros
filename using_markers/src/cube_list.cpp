#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  bool setColor = true;

  ros::init(argc, argv, "xube_list");
  ros::NodeHandle n;

  ros::Rate r(30);

  ros::Publisher pubTrackMarkers;
pubTrackMarkers = n.advertise<visualization_msgs::Marker>("/track_markers",1);
visualization_msgs::Marker markerMsg;

markerMsg.header.frame_id  = "/odom";
markerMsg.ns = "trackMarkers";
markerMsg.action = visualization_msgs::Marker::ADD;
markerMsg.pose.orientation.w = 1.0;
markerMsg.type = visualization_msgs::Marker::TRIANGLE_LIST;
markerMsg.scale.x = 0.8f;
markerMsg.scale.y = 0.8f;
markerMsg.scale.z = 0.8f;
if (setColor){
    markerMsg.color.r = 0;
    markerMsg.color.g = 1;
    markerMsg.color.b = 0;
    markerMsg.color.a = 1;
}

while(ros::ok()){

for (int i=0; i<12; i++){
    geometry_msgs::Point temp;
    temp.x = i;
    temp.y = 1;
    temp.z = 0.5;
    markerMsg.points.push_back(temp);
    temp.y += 1.0;
    markerMsg.points.push_back(temp);
    temp.x += 1.0;
    markerMsg.points.push_back(temp);
    std_msgs::ColorRGBA c;
    c.r = (float)i/10.0;
    c.g = 0;
    c.b = 0;
    c.a = 1;
    markerMsg.colors.push_back(c);
    markerMsg.colors.push_back(c);
    markerMsg.colors.push_back(c);
}

pubTrackMarkers.publish(markerMsg);
markerMsg.points.clear();
markerMsg.colors.clear();

r.sleep();
}

}