#include "follow_path.h"

void keyboarddownCallback(const keyboard::Key::ConstPtr& key){
  //ROS_INFO("pushed key num:[%d]",key->code);
  g_lastPushedKey = key->code;
  
  if(key->code == ENTER){
    g_followPath = true;
    if(!pathHistory.empty()){
      g_nextTargetPathNum = pathHistory.size() - 1;
    }
  }
  else if(key->code == SPACE){
    g_followPath = false;
  }
}

void keyboardupCallback(const keyboard::Key::ConstPtr& key){
  //ROS_INFO("key up");
  g_lastPushedKey = 0;
}

void chatterCallback(const std_msgs::Float32MultiArray& msg){
  g_gridPotantial[0] =  msg.data[0];
  g_gridPotantial[1] =  msg.data[1];
}

// followPathフラグ:オフ→経路の記録 :オン→次に進むポイントの指示
void trajectryCallback(const geometry_msgs::PoseStamped& trajectry){

  float X_pos = trajectry.pose.position.x;
  float Y_pos = trajectry.pose.position.y;
  float Yaw_ang = -trajectry.pose.orientation.z * M_PI;

  // 角度履歴記録用
  static int targetVectorAngRevState = 0;
  static int yawAngRevState = 0;

  if(g_followPath){
    // 次の目標点まで近づいていれば次の点を指定
    if( pow( pathHistory[g_nextTargetPathNum][0]-X_pos , 2) + pow( pathHistory[g_nextTargetPathNum][1]-Y_pos , 2) < APPROARCH_THRESHOLD*APPROARCH_THRESHOLD && g_nextTargetPathNum > 0){
      g_nextTargetPathNum--;
    }
    float targetVectorAng = -atan2f(pathHistory[g_nextTargetPathNum][1]-Y_pos,pathHistory[g_nextTargetPathNum][0]-X_pos);

    // 角度が-pi,piの間で非連続になる問題への対処
    if( targetVectorAng > M_PI/2.0 && targetVectorAngRevState != -1 ){ targetVectorAngRevState = 1; }
    else if(targetVectorAng < -M_PI/2.0 && targetVectorAngRevState == 1){ targetVectorAngRevState = 1; }
    else if(targetVectorAng < -M_PI/2.0 && targetVectorAngRevState != 1){ targetVectorAngRevState = -1; }
    else if(targetVectorAng > M_PI/2.0 && targetVectorAngRevState == -1){ targetVectorAngRevState = -1;}
    else{ targetVectorAngRevState = 0; }

    if( targetVectorAngRevState == 1 && targetVectorAng < -M_PI / 2.0 ){ targetVectorAng += 2.0 * M_PI; }
    else if(targetVectorAngRevState == -1 && targetVectorAng > M_PI / 2.0){ targetVectorAng -= 2.0 * M_PI; }

    if( Yaw_ang > M_PI/2.0 && yawAngRevState != -1 ){ yawAngRevState = 1; }
    else if( Yaw_ang < -M_PI/2.0 && yawAngRevState == 1){ yawAngRevState = 1; }
    else if( Yaw_ang < -M_PI/2.0 && yawAngRevState != 1){ yawAngRevState = -1; }
    else if( Yaw_ang > M_PI/2.0 && yawAngRevState == -1){ yawAngRevState = -1; }
    else{ yawAngRevState = 0; }

    if( yawAngRevState == 1 && Yaw_ang < -M_PI / 2.0 ){ Yaw_ang += 2.0 * M_PI; }
    else if(yawAngRevState == -1 && Yaw_ang > M_PI / 2.0){ Yaw_ang -= 2.0 * M_PI; }
    ////////////////////////////////////////////////////

    g_angleToTargetPoint = targetVectorAng - Yaw_ang;

    ROS_INFO("nowPos:(%f,%f),targetPos:(%f,%f),absTargetAngle:%f,wheelchairAng:%f,targetAng:%f",X_pos,Y_pos,pathHistory[g_nextTargetPathNum][0],pathHistory[g_nextTargetPathNum][1],targetVectorAng,Yaw_ang,g_angleToTargetPoint);

    // ゴールに近づいたら終了
    if(g_nextTargetPathNum <= 1){
      g_followPath = false;
    }
  }
  else{
    // 前に記録した点から1ｍ以上離れていれば新しく記録。何も記録されていなければ無条件で記録。
    if(pathHistory.empty()){
      std::vector<float> pos;
      pos.push_back(X_pos);
      pos.push_back(Y_pos);
      pathHistory.push_back(pos);
    }
    else{
      int last = pathHistory.size() - 1;
      if( pow( pathHistory[last][0] - X_pos , 2) + pow( pathHistory[last][1] - Y_pos , 2) > RECORD_THRESHOLD * RECORD_THRESHOLD ){
        std::vector<float> pos;
        pos.push_back(X_pos);
        pos.push_back(Y_pos);
        pathHistory.push_back(pos);
        ROS_INFO("pushed:(%f,%f)",X_pos,Y_pos);
      }
    }
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "calculate_voltage");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("potential_vector_of_grid", 1000, chatterCallback);
  ros::Subscriber sub_key = n.subscribe("keyboard/keydown", 1000, keyboarddownCallback); //キーボード入力取得
  ros::Subscriber sub_key_up = n.subscribe("keyboard/keyup", 1000, keyboardupCallback); //キーボード入力取得
  ros::Subscriber sub_pos = n.subscribe("slam_out_pose",1000,trajectryCallback);

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

// 経路追従フラグON：次の目標へ向かう指令 経路追従フラグOFF：ポテンシャルを考慮しながらキーボード入力で進む
bool calculateVoltage(joystic_wheelchair::OutputVoltage::Request &req,joystic_wheelchair::OutputVoltage::Response &res){

  double vol_x = 2.5;
  double vol_y = 2.5;
  double tmp_potentialVector[2] = {g_gridPotantial[0],g_gridPotantial[1]};

  if(g_followPath){
    if(g_angleToTargetPoint > M_PI/2.0 - 0.2){
      vol_x = 4.0;
    }
    else if(g_angleToTargetPoint < -M_PI/2.0 + 0.2){
      vol_x = 1.0;
    }
    else{
      
      tmp_potentialVector[1] += 3.0 * cos(g_angleToTargetPoint);
      tmp_potentialVector[0] += 3.0 * sin(g_angleToTargetPoint);
      vol_x = 2.5 + tmp_potentialVector[0]*5;
      vol_y = 2.5 + tmp_potentialVector[1]*3;
      
      /*
      vol_y = 2.5 + 2.0 * cos(g_angleToTargetPoint);
      vol_x = 2.5 + 3.0 * sin(g_angleToTargetPoint);
      */
    }
  }
  else{    
    if(g_lastPushedKey == UP){
      /*
      tmp_potentialVector[1] += 3.0;
      vol_x = 2.5 + tmp_potentialVector[0]*5;
      vol_y = 2.5 + tmp_potentialVector[1]*3;
      */
      vol_y = 4.9;
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

  }

  //ROS_INFO("vol_x:%f,vol_y:%f,pV0:%f,pV1:%f",vol_x,vol_y,g_gridPotantial[0],g_gridPotantial[1]);
  
  clamp(&vol_x , 0.1 , 4.9);
  clamp(&vol_y , 0.1 , 4.9);

  res.vol_x = vol_x;
  res.vol_y = vol_y;

  return true;
}