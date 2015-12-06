#include "recive_from_arduino.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "recive_from_arduino");
  ros::NodeHandle n;

  distance_pub = n.advertise<std_msgs::Int32>("sonic_distance", 1000);

  pid_t result_pid;
  struct termios oldtio, newtio;

  std::string port;
  ros::param::get("arduino_port", port);

  fd = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) {
    perror(port.c_str());
    return (-1);
  }

  tcgetattr(fd, &oldtio); /* 現在のシリアルポートの設定を待避させる*/
  memset(&newtio, 0, sizeof(newtio));/* 新しいポートの設定の構造体をクリアする */

  //シリアルポート準備
  serial_init(fd);

  serial_recive();

  return 0;
}


// シリアルポートの初期化
void serial_init(int fd) {
  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  tio.c_cflag = CS8 | CLOCAL | CREAD;
  /*
   BAUDRATE: ボーレートの設定．cfsetispeed と cfsetospeed も使用できる．
   CS8     : 8n1 (8 ビット，ノンパリティ，ストップビット 1)
   CLOCAL  : ローカル接続，モデム制御なし
   CREAD   : 受信文字(receiving characters)を有効にする．
   */

  tio.c_cc[VTIME] = 0; /* キャラクタ間タイマは未使用 */

  /*
   ICANON  : カノニカル入力(行単位入力）を有効にする
   */
    tio.c_lflag = ICANON;

  /*
   IGNPAR  : パリティエラーのデータは無視する
   ICRNL   : CR を NL に対応させる(これを行わないと，他のコンピュータで
   CR を入力しても，入力が終りにならない)
   それ以外の設定では，デバイスは raw モードである(他の入力処理は行わない)
   */
   tio.c_iflag = IGNPAR | ICRNL;

  // ボーレートの設定
   cfsetispeed(&tio, BAUDRATE);
   cfsetospeed(&tio, BAUDRATE);
  // デバイスに設定を行う
   tcsetattr(fd, TCSANOW, &tio);
 }

void serial_recive() {

  char buf[BUFFSIZE];
  int count;
  std::string msg;
  int distance;

  ROS_INFO("Recive Start");

  //STOPになるまで無限ループ
  while (FALSE == STOP && ros::ok()) {
    memset(&buf, 0, sizeof(buf));
    count = read(fd, &buf, BUFFSIZE);
    //printf("%d ",count);
    if (count < 0) {
      fprintf(stdout, "CHILD:Could not read from serial port\n");
      STOP = TRUE;
    } else {
      //ROS_INFO("msg:%s num:%d",buf,count);
      msg = buf;
      if(msg.substr(0,1)=="D"){
        distance = std::atoi((msg.substr(1)).c_str());
        ROS_INFO("distance:%d mm",distance);

        std_msgs::Int32 distance_msg;
        distance_msg.data = distance;
        distance_pub.publish(distance_msg);
      }
  }
  }

  ROS_INFO("Recive End");
}