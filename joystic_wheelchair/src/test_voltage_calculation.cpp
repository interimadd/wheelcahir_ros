#include "ros/ros.h"
#include "joystic_wheelchair/OutputVoltage.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_voltage_calculation");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<joystic_wheelchair::OutputVoltage>("output_voltage");
  joystic_wheelchair::OutputVoltage srv;
  ros::Rate loop_rate(2);
  srv.request.order = "do";
  while(ros::ok()){
    if(client.call(srv)){
      int vol_x = int(srv.response.vol_x * 10);
      int vol_y = int(srv.response.vol_y * 10);
      ROS_INFO("x:%d,y:%d\n",vol_x,vol_y);
    }
  loop_rate.sleep();
  }

  return 0;
}