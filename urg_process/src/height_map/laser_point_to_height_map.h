#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud.h"
#//include "urg_process/VectorField.h"
#include "height_map.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <algorithm>


ros::Publisher voxcel_pub;
tf::TransformListener* listener;
Height_Map height_map(0,0,100,100,0.03);

void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
int main(int argc, char **argv);