#include <ros/ros.h>
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Point.h"
#include <cmath>
#include <math.h>
#include <vector>
#include "std_msgs/Float32MultiArray.h"

using namespace std;

ros::Publisher potential_pub;

void calculatePotential(nav_msgs::GridCells::ConstPtr& gridInfo,double *potentialVector);
void chatterCallback(const std_msgs::Float32MultiArray& msg);
void clamp(double* x,double min,double max);
int main(int argc, char **argv);