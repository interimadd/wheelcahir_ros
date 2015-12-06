#include <ros/ros.h>
#include "keyboard/Key.h"
#include <cmath>
#include <math.h>
#include <vector>
#include "joystic_wheelchair/OutputVoltage.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;

ros::Publisher voltage_pub;

const int UP = 273;
const int DOWN = 274;
const int RIGHT = 275;
const int LEFT = 276;

int g_lastPushedKey = 0;
double g_keyboardPotential[2];
double g_gridPotantial[2];

void chatterCallback(const std_msgs::Float32MultiArray& msg);
void keyboarddownCallback(const keyboard::Key::ConstPtr& key);
void keyboardupCallback(const keyboard::Key::ConstPtr& key);
void clamp(double* x,double min,double max);
int main(int argc, char **argv);
bool calculateVoltage(joystic_wheelchair::OutputVoltage::Request &req,joystic_wheelchair::OutputVoltage::Response &res);