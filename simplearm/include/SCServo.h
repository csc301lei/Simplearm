/*
 * SCServo.h
 * Ӳ��ͨ�Žӿ�
 * ����: 2016.8.25
 * ����: ̷����
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
	virtual int writeSCS(unsigned char *nDat, int nLen);//���nLen�ֽ�
	virtual int readSCS(unsigned char *nDat, int nLen);//����nLen�ֽ�
	virtual int writeSCS(unsigned char bDat);//���1�ֽ�
	virtual void flushSCS();//ˢ�½ӿڻ�����
public:
	Serial serial;
};

#endif
