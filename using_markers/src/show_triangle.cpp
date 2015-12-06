#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <urg_process/VectorField.h>

ros::Publisher pubTrackMarkers;
const float TRIANGLE_LENGTH = 0.4f;

const float MAX_STRENGTH = 4.0f;

const int POSE_SIZE = 5;
const float TRIANGLE_POS[POSE_SIZE][3]={
  {0.0f,-2.0f,-0.2f},
  {0.0f,-1.0f,-0.2f},
  {0.0f, 0.0f,-0.2f},
  {0.0f, 1.0f,-0.2f},
  {0.0f, 2.0f,-0.2f}
};


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
    p[i].z = -1.0f;
    triangle->points.push_back(p[i]);
    triangle->colors.push_back(color);
  }
}


int main( int argc, char** argv ){

  ros::init(argc, argv, "visualize_vector_field");
  ros::NodeHandle n;

  ros::Rate r(8);

  pubTrackMarkers = n.advertise<visualization_msgs::Marker>("/visualized_vector",1);

  visualization_msgs::Marker markerMsg;
  std_msgs::ColorRGBA c;

  c.r = 0.0f;
  c.g = 1.0f;
  c.b = 0.0f;
  c.a = 1.0f;

  float change_distace = 0.0f;

  while(ros::ok()){

    markerMsg.header.frame_id  = "velodyne";
    markerMsg.ns = "visualized_vector";
    markerMsg.action = visualization_msgs::Marker::ADD;
    markerMsg.pose.orientation.w = 1.0;
    markerMsg.type = visualization_msgs::Marker::TRIANGLE_LIST;
    markerMsg.scale.x = 1.0f;
    markerMsg.scale.y = 1.0f;
    markerMsg.scale.z = 1.0f;

    for (int j=0; j<5;j++){
      for(int i=0; i<POSE_SIZE; i++){
        createTriangle(TRIANGLE_POS[i][0]  + float(j*2) + change_distace,TRIANGLE_POS[i][1], TRIANGLE_POS[i][2], c, &markerMsg);
      }
    }

    pubTrackMarkers.publish(markerMsg);
    markerMsg.points.clear();
    markerMsg.colors.clear();

    change_distace += 0.2f;
    if(change_distace > 6.0f){
      change_distace = 0.0f;
    }

    r.sleep();
  }

  return 0;
}