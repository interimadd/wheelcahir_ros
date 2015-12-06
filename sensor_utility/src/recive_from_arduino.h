    #include <stdio.h>
    #include <stdlib.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <string.h>
    #include <termios.h>
    #include <time.h>
    #include <sys/wait.h>
    #include "ros/ros.h"
    #include "std_msgs/Int32.h"

    /* <asm/termbits.h> で定義されているボーレートの設定．これは
     <termios.h>からインクルードされる． */
    #define BAUDRATE B9600
    //Arduinoのソフトウェアシリアルでも使えるように9600bpsにする

    #define BUFFSIZE 256
    #define COULDNOTFORK -1

    #define FALSE 0
    #define TRUE 1

    static int fd = -1;
    volatile int STOP = FALSE;

    ros::Publisher distance_pub;

    /* Functions */
    void serial_init(int fd);
    void serial_recive();

    /* Main */
    int main(int argc, char **argv);
