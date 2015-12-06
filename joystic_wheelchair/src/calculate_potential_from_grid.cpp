#include "calculate_potential_from_grid.h"

// 危険度ポテンシャルの勾配の計算
// U=1/rのポテンシャル場を定義し、点が密集している場所でのポテンシャルが大きくなりすぎる問題を、周囲の点の数をカウントし、その分で割ることにより対処している
void calculatePotential(const nav_msgs::GridCells::ConstPtr& gridInfo,double *potentialVector){
  
  potentialVector[0] = 0;
  potentialVector[1] = 0;

  int infoNum = gridInfo->cells.size();
  double powRadius = pow(0.15,2);

  for(int i = 0; i < infoNum; i++){

    double x = gridInfo->cells[i].x;
    double y = gridInfo->cells[i].y;
    //potentialVector[0] -= 2.0 * exp(-pow(x,2)-pow(y,2)) * x;
    //potentialVector[1] -= 2.0 * exp(-pow(x,2)-pow(y,2)) * y;
    int nearPointNum = 0;

    for(int n=0;n<infoNum;n++){
    if( powRadius > pow(gridInfo->cells[n].x - x,2) + pow(gridInfo->cells[n].y - y,2)){
        nearPointNum++;
      }
    }
    potentialVector[0] += x / pow( 4.0*( pow(x,2) + pow(y,2) ) , 1.5) / double(nearPointNum);
    potentialVector[1] -= y / pow( 4.0*( pow(x,2) + pow(y,2) ) , 1.5) / double(nearPointNum);
    //potentialVector[0] -= x / pow( ( pow(x,2) + pow(y,2) ) , 1.5);
    //potentialVector[1] -= y / pow( ( pow(x,2) + pow(y,2) ) , 1.5);
  }
}

void chatterCallback(const nav_msgs::GridCells::ConstPtr& msg){
  double potentialVector[2] = {0};
  calculatePotential(msg, potentialVector);
  ROS_INFO("x:%f,y:%f",potentialVector[0],potentialVector[1]);
  std_msgs::Float32MultiArray potential_data;
  potential_data.data.push_back(potentialVector[0]);
  potential_data.data.push_back(potentialVector[1]);
  potential_pub.publish(potential_data);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "calculate_potential_from_grid");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("laser_to_grid", 1000, chatterCallback);
  potential_pub = n.advertise<std_msgs::Float32MultiArray>("potential_vector_of_grid", 1000);
  
  ros::spin();
  return 0;
}
// %EndTag(FULLTEXT)%

void clamp(double* x,double min,double max){
  if(*x < min){
    *x = min;
  }
  else if(*x > max){
    *x = max;
  }
}
