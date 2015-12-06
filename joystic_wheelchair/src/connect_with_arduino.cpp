#include "connect_with_arduino.h"


void keyboarddownCallback(const keyboard::Key::ConstPtr& key){
  ROS_INFO("pushed key num:[%d]",key->code);
  switch(key->code){
    case 275:
      sendVoltageInput(0, 40);
      break;
    case 276:
      sendVoltageInput(0, 10);
      break;
    case 273: // ↑キー
      sendVoltageInput(1, 40);
      break;
    case 274:
      sendVoltageInput(1, 10);
      break;
  }
}

void keyboardupCallback(const keyboard::Key::ConstPtr& key){
  ROS_INFO("key up");
  if(key->code == 275 || key->code == 276){
    sendVoltageInput(0, 25);
  }
  else if(key->code == 273 || key->code == 274){
    sendVoltageInput(1, 25);
  }
  else{
    sendVoltageInput(0, 25);
    sendVoltageInput(1, 25);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "connect_with_arduino");
  ros::NodeHandle n;

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

  sleep(2);

  fprintf(stderr, "serial send start");
            
  ros::Subscriber sub_key = n.subscribe("keyboard/keydown", 1000, keyboarddownCallback); //キーボード入力取得
  ros::Subscriber sub_key_up = n.subscribe("keyboard/keyup", 1000, keyboardupCallback); //キーボード入力取得
  ros::spin();
  
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


// Xに3.0Vの電圧指令を送りたい→sendVoltageInput(0,30)
int sendVoltageInput(int XorY,int voltage){

  char input[6];

  if(XorY == 0){
    if( 10 <= voltage && voltage < 51){ sprintf(input,"VX%d\n",voltage);}
    else if( 0 <= voltage && voltage < 10){ sprintf(input,"VX0%d\n",voltage);}
  }
  else if(XorY == 1){
    if( 10 <= voltage && voltage < 51){ sprintf(input,"VY%d\n",voltage);}
    else if( 0 <= voltage && voltage < 10){ sprintf(input,"VY0%d\n",voltage);}
  }

  write(fd,&input,6);

  return 1;
}