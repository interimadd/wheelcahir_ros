#include "laser_to_grid.h"

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int grid_tmp[10000] = {-1};
	int gridIndex = 0;

	nav_msgs::GridCells grid;
	grid.cell_width = maxGridNumX;
	grid.cell_height = maxGridNumY;

  	int laserValueNum = int((msg->angle_max - msg->angle_min)/msg->angle_increment);

  	// 全ての点をグリッドマップの上に射影
  	// グリッドマップの定義内にある点群をグリッドマップ上のどの番号に相等するのかを計算し、配列に格納
  	for(int i = 0 ;i<laserValueNum;i++){
    	float rad = msg->angle_min + msg->angle_increment * i;
    	float y = msg->ranges[i] * cos(rad);
    	float x = msg->ranges[i] * sin(rad);
    	if(in_the_gridmap_or_not(x, y)){
    		int gridCol = int(x/gridLength) + maxGridNumX/2;
    		int gridRow = int(y/gridLength);
    		grid_tmp[gridIndex] = gridCol + gridRow * maxGridNumX;
    		gridIndex++;
    	}
  	}

  	// 重複している点を除くため、並び替えで昇順に
  	sort(grid_tmp,grid_tmp+gridIndex);

  	// 重複している点を無視して、グリッド座標ベクトルにプッシュしていく
  	for(int i = 0;i<gridIndex;i++){
  		if(grid_tmp[i] != grid_tmp[i+1]){
  			float y = grid_tmp[i]/maxGridNumX;
  			float x = grid_tmp[i] - y * maxGridNumX - maxGridNumX/2;
  			y = y * gridLength;
  			x = x * gridLength;

  			geometry_msgs::Point p;
  			p.x = x;
  			p.y = y;
  			p.z = 0; 
  			grid.cells.push_back(p);
  		}
  	}

  	grid_pub.publish(grid);
}	


int in_the_gridmap_or_not(float x,float y)
{
  const float DETECT_X_RANGE = maxGridNumX*gridLength/2.0f;
  const float DETECT_Y_RANGE = maxGridNumY*gridLength;
  if( abs(x)<DETECT_X_RANGE && 0 < y && y < DETECT_Y_RANGE ){ return 1; }
  else{ return 0; }
}



int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "laser_to_grid");

  ros::NodeHandle n;

  grid_pub = n.advertise<nav_msgs::GridCells>("laser_to_grid", 1000);
  ros::Subscriber sub = n.subscribe("scan", 1000, chatterCallback);

  ros::spin();

  return 0;
}