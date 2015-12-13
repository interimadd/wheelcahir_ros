#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <urg_process/VectorField.h>

ros::Publisher pubTrackMarkers;
const float TRIANGLE_LENGTH = 0.03f;

const float MAX_STRENGTH = 4.0f;

const int TABLE_SIZE = 20;
// 波長,R,G,B
const float RGB_TABLE[TABLE_SIZE][4] = 
{
    { 420.0f ,0.31129f ,0.00000f ,0.60684f },
    { 430.0f ,0.39930f ,0.00000f ,0.80505f },
    { 440.0f ,0.40542f ,0.00000f ,0.87684f },
    { 450.0f ,0.34444f ,0.00000f ,0.88080f },
    { 460.0f ,0.11139f ,0.00000f ,0.86037f },
    { 470.0f ,0.00000f ,0.15233f ,0.77928f },
    { 480.0f ,0.00000f ,0.38550f ,0.65217f },
    { 490.0f ,0.00000f ,0.49412f ,0.51919f },
    { 500.0f ,0.00000f ,0.59271f ,0.40008f },
    { 510.0f ,0.00000f ,0.69549f ,0.25749f },
    { 520.0f ,0.00000f ,0.77773f ,0.00000f },
    { 530.0f ,0.00000f ,0.81692f ,0.00000f },
    { 540.0f ,0.00000f ,0.82625f ,0.00000f },
    { 550.0f ,0.00000f ,0.81204f ,0.00000f },
    { 560.0f ,0.47369f ,0.77626f ,0.00000f },
    { 570.0f ,0.70174f ,0.71523f ,0.00000f },
    { 580.0f ,0.84922f ,0.62468f ,0.00000f },
    { 590.0f ,0.94726f ,0.49713f ,0.00000f },
    { 600.0f ,0.99803f ,0.31072f ,0.00000f },
    { 610.0f ,1.00000f ,0.00000f ,0.00000f },
};
const int STRENGTH_TABLE_SiZE = 6;
//sin5',sin10',sin15',sin20',0.075m,0.10mの値
const float STRENGTH_TABLE[STRENGTH_TABLE_SiZE]={
  0.085f,0.17f,0.255f,0.34f,1.4f,1.9f
} 

void strengthToRGB(float input_strength,std_msgs::ColorRGBA* c){

  int tableNum = 0;
  for(int i=0;i<STRENGTH_TABLE_SiZE;i++){
    if(input_strength < STRENGTH_TABLE[i]){
      tableNum = (i+1)*3;
      break;
    }
  }
  c->r = RGB_TABLE[tableNum][1];
  c->g = RGB_TABLE[tableNum][2];
  c->b = RGB_TABLE[tableNum][3];
  c->a = 1.0f;
}

void rotatePoint(geometry_msgs::Point *point,float yaw){
  float temp_x = point->x;
  float temp_y = point->y;
  point->x = cos(yaw)*temp_x - sin(yaw)*temp_y;
  point->y = sin(yaw)*temp_x + cos(yaw)*temp_y;
}
// (x,y)にyaw角度を持った三角形を表示する
void createTriangle(float x,float y,float yaw, std_msgs::ColorRGBA color,visualization_msgs::Marker *triangle){
  geometry_msgs::Point p[3];
  p[0].x =  TRIANGLE_LENGTH; p[0].y =  0.0f; 
  p[1].x = -TRIANGLE_LENGTH; p[1].y =  TRIANGLE_LENGTH/2.0f; 
  p[2].x = -TRIANGLE_LENGTH; p[2].y = -TRIANGLE_LENGTH/2.0f; 
  for(int i=0;i<3;i++){
    rotatePoint(&p[i],yaw);
    p[i].x += x;
    p[i].y += y;
    triangle->points.push_back(p[i]);
    triangle->colors.push_back(color);
  }
}

void dumpCallback(const urg_process::VectorField::ConstPtr& msg){

  visualization_msgs::Marker markerMsg;
  markerMsg.header.frame_id  = msg->child_frame_id;
  markerMsg.ns = "visualized_vector";
  markerMsg.action = visualization_msgs::Marker::ADD;
  markerMsg.pose.orientation.w = 1.0;
  markerMsg.type = visualization_msgs::Marker::TRIANGLE_LIST;
  markerMsg.scale.x = 1.0f;
  markerMsg.scale.y = 1.0f;
  markerMsg.scale.z = 1.0f;

  std_msgs::ColorRGBA c;

  for(int i=0; i<msg->data_num; i++){
    float vector_length = sqrt(msg->vec[i].x * msg->vec[i].x + msg->vec[i].y * msg->vec[i].y);
    strengthToRGB(vector_length,&c);
    createTriangle(msg->pos[i].x, msg->pos[i].y, atan2(msg->vec[i].y , msg->vec[i].x), c, &markerMsg);
    //ROS_INFO("%f,%f,%f",msg->pos[i].x,msg->pos[i].y,atan2(msg->vec[i].y , msg->vec[i].x) );
  }

  pubTrackMarkers.publish(markerMsg);
  markerMsg.points.clear();
  markerMsg.colors.clear();

  //int j = msg->data_num;
  //ROS_INFO("send_markeer_msg %d",j);

}

int main( int argc, char** argv ){

  ros::init(argc, argv, "visualize_vector_field");
  ros::NodeHandle n;

  ros::Rate r(30);

  pubTrackMarkers = n.advertise<visualization_msgs::Marker>("/visualized_vector",1);

  ros::Subscriber sub = n.subscribe("dump_gradient", 1000, dumpCallback);
  ros::spin();

  return 0;
}