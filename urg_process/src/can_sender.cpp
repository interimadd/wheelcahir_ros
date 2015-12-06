#include "can_sender.h"

int g_hardbit = 48;
int can_do = 0;

void chatterCallback(const std_msgs::Bool::ConstPtr& msg)
{
  char input[252];
  int inputcount = 0;
  int writecount = 0;

  //t 1FF 8 00 F0 FF FF FF FF FF FF \r
  input[0] = 116;

  input[1] = 49;
  input[2] = 70;
  input[3] = 70;
            
  input[4] = 56;

  g_hardbit++;
  if(g_hardbit>57){ g_hardbit = 48;}

  input[5] = 48;
  input[6] = g_hardbit;
  //input[6] = 48;

  input[7] = 48;
  input[8] = 70;

  input[9] = 70;
  input[10] = 70;

  input[11] = 70;
  input[12] = 70;

  input[13] = 70;
  input[14] = 70;

  input[15] = 70;
  input[16] = 70;

  input[17] = 70;
  input[18] = 70;

  input[19] = 70;
  input[20] = 70;

  input[21] =13;

 // stopflagが立っていたら停止のCAN信号を送る
  
  if(msg->data){
    input[7] = 70;
    input[8] = 48;
  }
  
  writecount = write(fd, &input, 22);

  if (writecount < 0) {
    fprintf(stdout, "Could not write to serial port %d\n", writecount);
  } else {
    fprintf(stdout, "Send %d bytes\n", writecount);
  }

if(msg->data){ ROS_INFO("send true");}
else{ROS_INFO("send false");}
  }

int main(int argc, char **argv)
{

        pid_t result_pid;

        struct termios oldtio, newtio;
        /*
         読み書きのためにモデムデバイスをオープンする．ノイズによって CTRL-C
         がたまたま発生しても接続が切れないように，tty 制御はしない．
         */

         fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY);
         if (fd < 0) {
          perror(MODEMDEVICE);
          return (-1);
        }

        tcgetattr(fd, &oldtio); /* 現在のシリアルポートの設定を待避させる*/
        memset(&newtio, 0, sizeof(newtio));/* 新しいポートの設定の構造体をクリアする */

        //シリアルポート準備
        serial_init(fd);

        can_open();

        ros::init(argc, argv, "can_sender");
        ros::NodeHandle n;


//forkして受信用と送信用に分ける
        result_pid = fork();

        if (result_pid == -1) {
            fprintf(stderr, "fork failed.\n");
            return COULDNOTFORK;
        }

        if (result_pid == 0) {
            //child_process();
            while(1){sleep(100);}
        } else {
            fprintf(stderr, "fork completed");

            //parent_process(result_pid);
            
            ros::Subscriber sub = n.subscribe("stopflag", 1000, chatterCallback);
            ros::spin();
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


    void parent_process(pid_t result_pid) {
       char input[BUFFSIZE];
        int writecount = 0;
        fprintf(stdout, "Parent:Waiting for Input\n");
        int i = 0;
        int inputcount = 0;
        while (1) {
            memset(&input, 13, sizeof(input));

            fgets(input, sizeof(input), stdin);
            fflush(stdin);

            //改行コード埋め込み
            for (i = 0; i < BUFFSIZE; i++) {
                if (input[i] == 0) {
                    inputcount = i;
                    input[i] = 13;
                    break;
                }
            }

            while(1){
            input[0] = 86;
            input[1] = 13;
            inputcount = 2;

            writecount = write(fd, &input, inputcount);

            //t 1FF 8 00 F0 FF FF FF FF FF FF \r
            input[0] = 116;

            input[1] = 49;
            input[2] = 70;
            input[3] = 70;
            
            input[4] = 56;

            input[5] = 48;
            input[6] = 48;

            input[7] = 70;
            input[8] = 48;

            input[9] = 70;
            input[10] = 70;

            input[11] = 70;
            input[12] = 70;

            input[13] = 70;
            input[14] = 70;

            input[15] = 70;
            input[16] = 70;

            input[17] = 70;
            input[18] = 70;

            input[19] = 70;
            input[20] = 70;

            input[21] =13;

/*
            while(1){
              writecount = write(fd, &input, 22);
              usleep(100000);
           }
*/
           writecount = write(fd, &input, 22);

            if (writecount < 0) {
                fprintf(stdout, "Could not write to serial port %d\n", writecount);
                break;
            } else {
                fprintf(stdout, "Send %d bytes\n", writecount);

            }
            usleep(10);
          }

        }
      /*
      char key;

      while(1){
        key = getchar();
        //if(key < ' ') break;
        switch (key){
          case 'r':
            write(fd,"\r",2);
            break;
          case 'f':
            write(fd,"F\r",3);
            break;
          case 'c':
            write(fd,"C\r",3);
            break;
          default:
            printf("key:%d\n\r",key);
            break;
        }
        usleep(100000);
      }

      printf("finish\n");
      */
    }

    void can_open(){
      // マニュアルに従って、最初は改行コードを送り、その後CANオープン
        char input[8];
        input[0] = 13;
        write(fd, &input, 1);
        write(fd, &input, 1);
        write(fd, &input, 1);
        write(fd, &input, 1);
        write(fd, &input, 1);

        // send "C"
        input[0] = 67;
        input[1] = 13;
        write(fd, &input, 2);

        // send "S6"
        input[0] = 83;
        input[1] = 54;
        input[2] = 13;
        write(fd, &input, 3);

        // send "O"
        input[0] = 79;
        input[1] = 13;
        write(fd, &input, 2);
    }