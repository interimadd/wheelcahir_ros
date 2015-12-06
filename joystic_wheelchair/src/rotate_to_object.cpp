#include <ros/ros.h>
#include <vector>
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include "joystic_wheelchair/OutputVoltage.h"

using namespace std;

const float DETECT_LENGTH = 0.7;
const float ROTATE_THRESHOLD = 20.0   * M_PI/180.0;

const int TURN_RIGHT =  1;
const int DEFAULT 	 =  0;
const int TURN_LEFT  = -1;

int g_whichToRotate = DEFAULT;


//前方1ｍの円周上に存在する点の重心をとり、大きい方に観点するフラグを立てる
void LRFCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  
  std::vector<float> detectAngle;

  int laserPointNum = int((msg->angle_max - msg->angle_min)/msg->angle_increment);

  // 指定範囲内に点があれば角度を記録
  for(int i = 0 ; i < laserPointNum - 1 ; i++){
  	if( msg->ranges[i] < DETECT_LENGTH){
  		 float rad = msg->angle_min + msg->angle_increment * i;
  		 detectAngle.push_back(rad);
  	}
  }

  if(detectAngle.empty()){
  	g_whichToRotate = DEFAULT;
  }
  else{
  	float sum = 0;
  	float meanOfDetectPoint = 0;
  	for(int i = 0; i < detectAngle.size() ; i++){
  		sum += detectAngle[i];
  	}
  	meanOfDetectPoint = sum / detectAngle.size();

  	if(meanOfDetectPoint < -ROTATE_THRESHOLD){
  		g_whichToRotate = TURN_RIGHT;
  	}
  	else if(meanOfDetectPoint > ROTATE_THRESHOLD){
  		g_whichToRotate = TURN_LEFT;
  	}
  	else{
  		g_whichToRotate = DEFAULT;
  	}

  	ROS_INFO("meanOfDetectPonint:%f,whichTurn:%d",meanOfDetectPoint,g_whichToRotate);
  }

}



bool calculateVoltage(joystic_wheelchair::OutputVoltage::Request &req,joystic_wheelchair::OutputVoltage::Response &res){

  double vol_x = 2.5;
  double vol_y = 2.5;

  switch(g_whichToRotate){
  	case 1:
  		vol_x = 4.9 ;
  		break;
  	case -1:
  		vol_x = 0.1 ;
  		break;
  	default:
  		vol_x = 2.5;
  }

  res.vol_x = vol_x;
  res.vol_y = vol_y;

  return true;
}



int main( int argc, char** argv )
{
  ros::init(argc, argv, "rotate_to_object");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("scan", 1000, LRFCallback);

  ros::ServiceServer service = n.advertiseService("output_voltage", calculateVoltage);
  
  ros::spin();
  return 0;
}

