#include "send_voltage_to_arduino.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "connect_with_arduino");
  ros::NodeHandle n;

  pid_t result_pid;
  struct termios oldtio, newtio;

  std::string port;
  //ros::param::set("arduino_port", "/dev/ttyACM0");
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

  //forkして受信用と送信用に分ける
  result_pid = fork();

  if (result_pid == -1) {
    fprintf(stderr, "fork failed.\n");
    return COULDNOTFORK;
  }

  if (result_pid == 0) {
    //child_process();
    while(1){sleep(100);}
  } 
  else{
    fprintf(stderr, "fork completed");
            
    //ros::Subscriber sub_key = n.subscribe("keyboard/keydown", 1000, keyboarddownCallback); //キーボード入力取得
    //ros::Subscriber sub_key_up = n.subscribe("keyboard/keyup", 1000, keyboardupCallback); //キーボード入力取得
    //ros::Subscriber voltage = n.subscribe("voltage", 1000, volatageCallback);
    ros::ServiceClient client = n.serviceClient<joystic_wheelchair::OutputVoltage>("output_voltage");
    joystic_wheelchair::OutputVoltage srv;
    ros::Rate loop_rate(5);
    srv.request.order = "do";
    while(ros::ok()){
      if(client.call(srv)){
        int vol_x = int(srv.response.vol_x * 10);
        int vol_y = int(srv.response.vol_y * 10);
        sendVoltageInput(X, vol_x);
        sendVoltageInput(Y, vol_y);
        ROS_INFO("x:%d,y:%d\n",vol_x,vol_y);
      }
      loop_rate.sleep();
    }
  }

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

void child_process() {

        char buf[BUFFSIZE];
        int count;

        fprintf(stdout, "CHILD:RCV Start\n");

        //STOPになるまで無限ループ
        while (FALSE == STOP) {
          memset(&buf, 0, sizeof(buf));
          count = read(fd, &buf, BUFFSIZE);
          printf("%d ",count);
          if (count < 0) {
            fprintf(stdout, "CHILD:Could not read from serial port\n");
            STOP = TRUE;
          } else {
            fprintf(stdout, "CHILD:RCVD CHAR %s %d \n", buf, count);

   }
  }

fprintf(stdout, "CHILD:BYE!\n");
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