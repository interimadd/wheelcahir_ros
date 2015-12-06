#include "calculate_potential.h"

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

void keyboarddownCallback(const keyboard::Key::ConstPtr& key){
  ROS_INFO("pushed key num:[%d]",key->code);
  g_lastPushedKey = key->code;
}

void keyboardupCallback(const keyboard::Key::ConstPtr& key){
  ROS_INFO("key up");
  g_lastPushedKey = 0;
}

void chatterCallback(const nav_msgs::GridCells::ConstPtr& msg){
  double potentialVector[2] = {0};
  calculatePotential(msg, potentialVector);
  ROS_INFO("x:%f,y:%f",potentialVector[0],potentialVector[1]);
  g_gridPotantial[0] =  potentialVector[0];
  g_gridPotantial[1] =  potentialVector[1];
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "calculate_potential");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("laser_to_grid", 1000, chatterCallback);
  ros::Subscriber sub_key = n.subscribe("keyboard/keydown", 1000, keyboarddownCallback); //キーボード入力取得
  ros::Subscriber sub_key_up = n.subscribe("keyboard/keyup", 1000, keyboardupCallback); //キーボード入力取得

  ros::ServiceServer service = n.advertiseService("output_voltage", calculateVoltage);
  
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

bool calculateVoltage(joystic_wheelchair::OutputVoltage::Request &req,joystic_wheelchair::OutputVoltage::Response &res){

  double vol_x = 2.4;
  double vol_y = 2.4;
  double tmp_potentialVector[2] = {g_gridPotantial[0],g_gridPotantial[1]};    

  if(g_lastPushedKey == UP){
    tmp_potentialVector[1] += 3.0;
    vol_x = 2.4 + tmp_potentialVector[0]*5;
    vol_y = 2.4 + tmp_potentialVector[1]*3;
  }
  else if(g_lastPushedKey == DOWN){
    vol_y = 0.1;
  }
  else if(g_lastPushedKey == RIGHT){
    vol_x = 4.9;
  }
  else if(g_lastPushedKey == LEFT){
    vol_x = 0.1;
  }
  
  clamp(&vol_x , 0.1 , 4.9);
  clamp(&vol_y , 0.1 , 4.9);

  res.vol_x = vol_x;
  res.vol_y = vol_y;

  return true;
}