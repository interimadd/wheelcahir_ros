#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

class ScanToPoint{
  private:
    laser_geometry::LaserProjection projector_;
    ros::Publisher pubTrackMarkers;
    ros::NodeHandle n;
    ros::Subscriber sub;
    tf::TransformListener listener_;
  public:
    ScanToPoint(int argc,char** argv);
    void start();
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
};


void ScanToPoint::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/odom",
        scan_in->header.stamp + ros::Duration(1.0),
        ros::Duration(1.0))){
     return;
  }

  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud("/odom",*scan_in,cloud,listener_);

  pubTrackMarkers.publish(cloud);

  // Do something with cloud.
}

ScanToPoint::ScanToPoint(int argc,char** argv){
  pubTrackMarkers = n.advertise<sensor_msgs::PointCloud>("/laser_point",1);
}

void ScanToPoint::start(){
  sub = n.subscribe("scan", 1000,&ScanToPoint::scanCallback,this);
  ros::spin();
}

int main( int argc, char** argv ){
  ros::init(argc, argv, "lase_scan_to_point_cloud");
  ScanToPoint manager(argc,argv);
  manager.start();
  return 0;
}