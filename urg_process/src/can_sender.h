    #include <stdio.h>
    #include <stdlib.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <string.h>
    #include <termios.h>
    #include <time.h>
    #include <sys/wait.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"

    /* <asm/termbits.h> で定義されているボーレートの設定．これは
     <termios.h>からインクルードされる． */
    #define BAUDRATE B9600
    //Arduinoのソフトウェアシリアルでも使えるように9600bpsにする

    /* 適切なシリアルポートを指すように，この定義を変更
     * 我が家の環境ではArduinoは/dev/ttyACM0になってました*/
    #define MODEMDEVICE "/dev/ttyUSB0"

    #define BUFFSIZE 256
    #define COULDNOTFORK -1

    #define FALSE 0
    #define TRUE 1

	#define X 0
	#define Y 1

    volatile int STOP = FALSE;
    static int fd = -1;

    /* Functions */
    void serial_init(int fd);
    void child_process();
    void parent_process(pid_t result_pid);
    void can_open();

    /* Main */
    int main(int argc, char **argv);
