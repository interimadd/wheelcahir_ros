#include "calculate_voltage_from_potential_and_keyboard.h"

void keyboarddownCallback(const keyboard::Key::ConstPtr& key){
  ROS_INFO("pushed key num:[%d]",key->code);
  g_lastPushedKey = key->code;
}

void keyboardupCallback(const keyboard::Key::ConstPtr& key){
  ROS_INFO("key up");
  g_lastPushedKey = 0;
}

void chatterCallback(const std_msgs::Float32MultiArray& msg){
  g_gridPotantial[0] =  msg.data[0];
  g_gridPotantial[1] =  msg.data[1];
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "calculate_voltage");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("potential_vector_of_grid", 1000, chatterCallback);
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