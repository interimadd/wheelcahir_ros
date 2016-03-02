#include "road_suface_analizer.h"

class ROAD_SURFACE_ANALIZER(){
	
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "road_suface_analizer");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("laser_to_grid", 1000, chatterCallback);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  
  ros::spin();
}