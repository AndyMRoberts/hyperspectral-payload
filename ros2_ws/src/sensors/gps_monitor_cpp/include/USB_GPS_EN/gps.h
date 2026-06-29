#ifndef _GPS_H_
#define _GPS_H_

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

void read_GPS_Data(char *Gps_Buffer);
void parse_GpsDATA();
void print_Save();
void RST_Buffer(char *Buffer);
void insert_array(char *buff);



struct
{
  char GPS_DATA[80];
  char GetData_Flag; //获取到 GPS 数据标志位
  char ParseData_Flag; //解析完成标志位
  char UTCTime[12]; //UTC 时间
  char Slatitude[15]; //纬度
  char N_S[2]; //N/S
  char Slongitude[15]; //经度
  char E_W[2]; //E/W
  char Usefull_Flag; //定位信息是否有效标志位
} Save_Data;



void read_GPS_Data(char *Gps_Buffer)
{
  char* GPS_DATAHead;
  char* GPS_DATATail;
  if ((GPS_DATAHead = strstr(Gps_Buffer, "$GPRMC,")) != NULL || (GPS_DATAHead =strstr(Gps_Buffer, "$GNRMC")) != NULL ){
    if (((GPS_DATATail = strstr(GPS_DATAHead, "\n")) != NULL) && (GPS_DATATail >GPS_DATAHead)){
      memset(Save_Data.GPS_DATA, 0, strlen(Save_Data.GPS_DATA));
      memcpy(Save_Data.GPS_DATA, GPS_DATAHead, GPS_DATATail - GPS_DATAHead);
      Save_Data.GetData_Flag = 1;
    }
  }
}

void parse_GpsDATA()
{
  char *subString;
  char *subStringNext;
  if (Save_Data.GetData_Flag){
    Save_Data.GetData_Flag = 0;

    //printf("%s\n",Save_Data.GPS_DATA);

    for (int i = 0 ; i <= 6 ; i++){
      if (i == 0){
        if ((subString = strstr(Save_Data.GPS_DATA, ",")) == NULL){
          printf("ERROR\n");//解析错误
        }
      }else{
        subString++;
        if ((subStringNext = strstr(subString, ",")) != NULL){
          // char buffer[12];
          char usefullBuffer[2];
          switch(i){
            case 1:RST_Buffer(Save_Data.UTCTime);
              memcpy(Save_Data.UTCTime, subString, subStringNext - subString);break; //获取 UTC 时间
            case 2:RST_Buffer(usefullBuffer);
              memcpy(usefullBuffer, subString, subStringNext - subString);break;//获取定位状态
            case 3:RST_Buffer(Save_Data.Slatitude);
              memcpy(Save_Data.Slatitude, subString, subStringNext - subString);break;
            case 4:RST_Buffer(Save_Data.N_S);
              memcpy(Save_Data.N_S, subString, subStringNext - subString);break;//获取 N/S
            case 5:RST_Buffer(Save_Data.Slongitude);
              memcpy(Save_Data.Slongitude, subString, subStringNext - subString);break;
            case 6:RST_Buffer(Save_Data.E_W);
              memcpy(Save_Data.E_W, subString, subStringNext - subString);break;//获取 E/W
            default:break;
          }
          subString = subStringNext;
          Save_Data.ParseData_Flag = 1;
          if(usefullBuffer[0] == 'A')
            Save_Data.Usefull_Flag = 1;
          else 
            if(usefullBuffer[0] == 'V')
              Save_Data.Usefull_Flag = 0;
        }else{
          printf("ERROR\n");//解析错误
        }
      }
    }
  }
}

void print_Save()
{
  printf("*****************************************************\n");
  printf("UTCTime\t\t:[%s]\n",Save_Data.UTCTime);
  printf("Slatitude\t:[%s]\n",Save_Data.Slatitude);
  printf("N/S\t\t:[%s]\n",Save_Data.N_S);
  printf("Slongitude\t:[%s]\n",Save_Data.Slongitude);
  printf("E/W\t\t:[%s]\n",Save_Data.E_W);
  insert_array(Save_Data.Slatitude);
  insert_array(Save_Data.Slongitude);
  printf("%s,%s\n",Save_Data.Slatitude, Save_Data.Slongitude);
  printf("*****************************************************\n");
}

void RST_Buffer(char *Buffer)
{
  memset(Buffer, 0, strlen(Buffer));
}

void insert_array(char *buff)
{
  char *p;
  int place = 0;
  char sbuffer[20]={0};
  p = strstr( buff , ".");
  place = p-buff;
  //printf("%d\n", place);
  for(int i=0; i<place-2; i++){
    sbuffer[i]=buff[i];
  }
  sbuffer[place-2]=' ';
  for(int i= place-1; buff[i-1]!='\0'; i++){
    sbuffer[i]=buff[i-1];
  }
  for(int i=0; sbuffer[i]!='\0'; i++){
    buff[i]=sbuffer[i];
  }
}



 


#endif
