#ifndef _COM_H_
#define _COM_H_
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#define BUFFER_SIZE 600      /*最大缓存区*/
 
int set_com_config(int fd,int baud_rate,int data_bits, char parity, int stop_bits);
int open_port( char *dev);
void delayms(int data);
int read_Buffer(int Sfd, char *Sread_buffer);


 
/****************************************/
/************打开串口函数****************/ 
/****************************************/
int open_port(char *dev) 
{
  int fd;
  
  printf("open dev [%s]\n",dev);
  //O_NDELAY 同 O_NONBLOCK。
  fd = open(dev, O_RDWR|O_NOCTTY); 
  if (fd < 0) { 
    perror("open serial port"); 
    return(-1); 
  }
  
  //恢复串口为阻塞状态 
  //非阻塞：fcntl(fd,F_SETFL,FNDELAY)
  //阻塞：fcntl(fd,F_SETFL,0)
  if (fcntl(fd, F_SETFL, 0) < 0)
  {
    perror("fcntl F_SETFL\n");
  }
  /*测试是否为终端设备*/ 
  if (isatty(STDIN_FILENO) == 0) {
    perror("standard input is not a terminal device");
  }
  return fd; 
}

/***********************************/
/************串口配置***************/
/***********************************/
int set_com_config(int fd,int baud_rate,int data_bits, char parity, int stop_bits)
{
  struct termios opt;
  int speed;
  if(tcgetattr(fd, &opt) != 0){
    perror("tcgetattr"); 
    return -1;
  }

  /* 设置字符大小*/
  //opt = opt;
  //cfmakeraw(&opt);//配置为原始模式
  //opt.c_cflag &= ~CSIZE;//c_cflag 控制模式标志

  /*设置波特率*/ 
  switch (baud_rate) 
  {
    case 2400:  speed = B2400;   break;
    case 4800:  speed = B4800;   break;
    case 9600:  speed = B9600;   break;
    case 19200: speed = B19200;  break;
    case 38400: speed = B38400;  break;
    default:    speed = B115200; break;
  }
  cfsetispeed(&opt, speed); 
  cfsetospeed(&opt, speed); 
  tcsetattr(fd,TCSANOW,&opt);

  opt.c_cflag &= ~CSIZE;

  /*设置数据位*/ 
  switch (data_bits)
  {
    case 7: {opt.c_cflag |= CS7;}break;//7个数据位  
    default:{opt.c_cflag |= CS8;}break;//8个数据位 
  }

  /*设置奇偶校验位*/ 
  switch (parity) //N
  {
    case 'n':case 'N': 
    {
      opt.c_cflag &= ~PARENB;//校验位使能
      opt.c_iflag &= ~INPCK; //奇偶校验使能
    }break;
    case 'o':case 'O': 
    {
      opt.c_cflag |= (PARODD | PARENB);//PARODD使用奇校验而不使用偶校验 
      opt.c_iflag |= INPCK;
    }break; 
    case 'e':case 'E': 
    { 
      opt.c_cflag |= PARENB;   
      opt.c_cflag &= ~PARODD;  
      opt.c_iflag |= INPCK;    
    }break;
    case 's':case 'S': /*as no parity*/  
    { 
      opt.c_cflag &= ~PARENB; 
      opt.c_cflag &= ~CSTOPB; 
    }break;
    default:
    {
      opt.c_cflag &= ~PARENB;//校验位使能     
      opt.c_iflag &= ~INPCK; //奇偶校验使能          	
    }break; 
  }

  /*设置停止位*/ 
  switch (stop_bits)
  {
    case 1: {opt.c_cflag &=  ~CSTOPB;} break;
    case 2: {opt.c_cflag |= CSTOPB;}   break;
    default:{opt.c_cflag &=  ~CSTOPB;} break;
  }

  /*处理未接收字符*/ 
  tcflush(fd, TCIFLUSH); 

  /*设置等待时间和最小接收字符*/ 
  opt.c_cc[VTIME]  = 11; 
  opt.c_cc[VMIN] = 0; 

  /*关闭串口回显*/
  opt.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHOK|ECHONL|NOFLSH); 
 
  /*激活新配置*/ 
  if((tcsetattr(fd, TCSANOW, &opt)) != 0){ 
    perror("tcsetattr"); 
    return -1; 
  }
  return 0; 
}

void delayms(int data)
{
  usleep(data*1000);
}


int read_Buffer(int Sfd, char *Sread_buffer)
{
  int ret = 0;
  int Sleng = 0;
  int times = 0;
  char rbuffer[32]={0};
  do{
    memset(rbuffer, 0, 32);
    Sleng = read(Sfd, rbuffer, 32);
    for(int j=0; j<Sleng; j++){
      if( times*32 + j >= BUFFER_SIZE)
        return -1;
      Sread_buffer[ times*32 + j] = rbuffer[j];
    }
    //printf("%s\n", rbuffer);
    ret = ret + Sleng;
    times++;
    delayms(20);
  }while(Sleng == 32);
  return ret;
}

#endif
