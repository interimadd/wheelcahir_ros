#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Point.h"
#include <math.h>
#include <algorithm>

using namespace std;

ros::Publisher grid_pub;

const int maxGridNumX = 40;
const int maxGridNumY = 40;

const float gridLength = 0.1;

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
int in_the_gridmap_or_not(float x,float y);
int main(int argc, char **argv);