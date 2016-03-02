#include "laser_point_to_voxcel_map.h"


void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg){

	tf::StampedTransform transform;
	
	try{
      listener->lookupTransform("/odom","/map_center",ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

	if( pow(voxcel_map.center_x-transform.getOrigin().x(),2) + pow(voxcel_map.center_y-transform.getOrigin().y(),2) > 1.0 ){
		voxcel_map.MoveVoxcelMapCenter(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
		//printf("%f,%f,%f\n",transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
	}

	voxcel_map.RecordSensorData(*msg);

	sensor_msgs::PointCloud send_point_cloud;
	send_point_cloud.header = msg->header;
	voxcel_map.VoxcelToPointCloud(&send_point_cloud);
	voxcel_pub.publish(send_point_cloud);

}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "laser_point_to_voxcel_map");
  ros::NodeHandle n;

  tf::TransformListener lr(ros::Duration(10));
  listener = &lr;

  voxcel_pub = n.advertise<sensor_msgs::PointCloud>("voxcel", 1000);
  ros::Subscriber sub = n.subscribe("laser_point", 1000, PointCloudCallback);

  ros::spin();

  return 0;
}