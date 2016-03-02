#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud.h"
#include "urg_process/VectorField.h"
#include "voxcel_map.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <algorithm>


ros::Publisher voxcel_pub;
tf::TransformListener* listener;
Voxcel_Map voxcel_map(2.0,0.0,100,50,40,0.03,0.005);

void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
int main(int argc, char **argv);