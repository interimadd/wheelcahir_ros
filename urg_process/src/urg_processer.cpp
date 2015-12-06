/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "keyboard/Key.h"
#include <math.h>

ros::Publisher flag_pub;
int g_last_pushed_key = 0;
int count = 0;

int hazard_or_not(float x,float y){
  const float DETECT_X_RANGE = 0.3;
  const float DETECT_Y_RANGE = 1.5;
  if( abs(x)<DETECT_X_RANGE && 0 < y && y < DETECT_Y_RANGE ){ return 1; }
  else{ return 0; }
}

void keyboardCallback(const keyboard::Key::ConstPtr& key){
  ROS_INFO("pushed key num:[%d]",key->code);
  g_last_pushed_key = key->code;
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //前方の±30度に障害物があるかどうかだけ検出
  //まず距離配列の中のどこからどこに相等するのかを決める
  int zeroDegNum = int((msg->angle_max - msg->angle_min)/msg->angle_increment/2.0);
  int rangeNum = int(45.0*3.14/180.0/msg->angle_increment);

  //この範囲の中で1m以下の点を数える
  int detectCount = 0;

  /*
  for(int i;i<rangeNum*2;i++){
    if(msg->ranges[zeroDegNum-rangeNum+i] < 2.5 ){
      detectCount++;
    }
  }
  */

  // 全ての点に対し、危険エリアに入っていないかチェック
  for(int i = 0 ;i<zeroDegNum*2-1;i++){
    float rad = msg->angle_min + msg->angle_increment * i;
    float y = msg->ranges[i] * cos(rad);
    float x = msg->ranges[i] * sin(rad);
    if(hazard_or_not(x, y)){
      detectCount++;
    }
  }

  //ROS_INFO("I heard: [%f]", msg->angle_min);
  ROS_INFO("counted Num:[%d]",detectCount);

  std_msgs::Bool flag;
  if(detectCount > 5 && g_last_pushed_key != 13){
    flag.data = true;
  }
  else{
    flag.data = false;
  }

  flag_pub.publish(flag);
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "urg_processer");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  flag_pub = n.advertise<std_msgs::Bool>("stopflag", 1000);
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("scan", 1000, chatterCallback);
  ros::Subscriber sub_key = n.subscribe("keyboard/keydown", 1000, keyboardCallback);
// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
