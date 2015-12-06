#include <ros/ros.h>
#include <cmath>
#include <math.h>
#include <vector>
#include "keyboard/Key.h"
#include "joystic_wheelchair/OutputVoltage.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;

ros::Publisher voltage_pub;

const float RECORD_THRESHOLD = 0.3; // 通過点の記録閾値
const float APPROARCH_THRESHOLD = 1.0; // 目標点への到達判定

const int UP = 273;
const int DOWN = 274;
const int RIGHT = 275;
const int LEFT = 276;
const int ENTER = 13;
const int SPACE = 32;

bool g_followPath = false; // 来た道を戻るモードか否か ENTERキーでオン SPACEキーでオフ

int g_lastPushedKey = 0;
double g_keyboardPotential[2];
double g_gridPotantial[2];

std::vector< std::vector<float> > pathHistory;
float g_angleToTargetPoint = 0;
int  g_nextTargetPathNum = 0;

void chatterCallback(const std_msgs::Float32MultiArray& msg);
void keyboarddownCallback(const keyboard::Key::ConstPtr& key);
void keyboardupCallback(const keyboard::Key::ConstPtr& key);
void trajectryCallback(const geometry_msgs::PoseStamped& trajectry);
void playTrajectry(const geometry_msgs::PoseStamped& trajectry);

void clamp(double* x,double min,double max);
int main(int argc, char **argv);
bool calculateVoltage(joystic_wheelchair::OutputVoltage::Request &req,joystic_wheelchair::OutputVoltage::Response &res);