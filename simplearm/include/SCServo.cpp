/*
 * SCServo.cpp
 * 硬件通信接口
 * 日期: 2016.8.9
 * 作者: 谭雄乐
 */

#include "string.h"
#include "SCServo.h"

SCServo::SCServo()
{
	FILE* output;
	const char script[] = "ls /dev/ttyUSB1";
	const char alarm[] = "ttyUSB";
	char buf[100];
	char* alarm_str;
	output = popen(script,"r");
	fread(buf,sizeof(char),sizeof(buf),output);
	pclose(output);
	alarm_str = strstr(buf,alarm);
	
	if(alarm_str!=NULL)
	{
		int i=0;
        while(buf[i])
        {
		   if(buf[i]==10)
		     buf[i]='\0';	
		   i++;
		}
		if(serial.Open(buf))
		{
		   serial.Set(1000000,8,'N',1);    	
		}
    }
    else
    {
	   printf("Serial Error\n");
    }
}

int SCServo::readSCS(unsigned char *nDat, int nLen)
{
	return serial.Read(nDat,nLen);
}

int SCServo::writeSCS(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	return serial.Write(nDat,nLen);
}

int SCServo::writeSCS(unsigned char bDat)
{
	return serial.Write(&bDat,1);
}

void SCServo::flushSCS()
{
	unsigned char temp;
	while(temp!=0)
	{
		serial.Read(&temp,1);
    }
}

const int Angle2pos1(double angle){
    int pos = 11.43 * angle - 0.4841;
    if(pos>1000) return 1000;
    else if(pos<0) return 0;
    else
	return pos;
}

const int Angle2pos2(double angle){
    int pos = 11.52 * angle - 17.34;
    if(pos>1000) return 1000;
    else if(pos<0) return 0;
    else
	return pos;
}

const int Angle2pos3(double angle){
    int pos = 11.47 * angle - 19.94;
    if(pos>1000) return 1000;
    else if(pos<0) return 0;
    else
	return pos;
}

