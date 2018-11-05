/*
 * SCServo.h
 * 硬件通信接口
 * 日期: 2016.8.25
 * 作者: 谭雄乐
 */

#ifndef _SCSERVO_H
#define _SCSERVO_H

#include "Serial.h"
#include "SCSProtocol.h"
#include "SCSProtocol.cpp"


const int Angle2pos1(double angle);
const int Angle2pos2(double angle);
const int Angle2pos3(double angle);


class SCServo : public SCSProtocol
{
public:
	SCServo(void);
	virtual int writeSCS(unsigned char *nDat, int nLen);//输出nLen字节
	virtual int readSCS(unsigned char *nDat, int nLen);//输入nLen字节
	virtual int writeSCS(unsigned char bDat);//输出1字节
	virtual void flushSCS();//刷新接口缓冲区
public:
	Serial serial;
};

#endif
