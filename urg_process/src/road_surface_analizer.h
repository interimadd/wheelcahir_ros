#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Point.h"
#include <cmath>

using namespace std;

class ROAD_SURFACE_ANALIZER(){

	public:

	private:

};

void chatterCallback(const nav_msgs::GridCells::ConstPtr& msg);
int main(int argc, char **argv);