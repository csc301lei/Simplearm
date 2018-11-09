/*
 * SCServo.h
 * Ӳ��ͨ�Žӿ�
 * ����: 2016.8.25
 * ����: ̷����
 */

#ifndef _SCSERVO_H
#define _SCSERVO_H

#include <pthread.h>
#include <ros/ros.h>
#include <sched.h>
#include <serial/serial.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include "SCSProtocol.cpp"
#include "SCSProtocol.h"
//#include "Serial.h"

const int Angle2pos1(double angle);
const int Angle2pos2(double angle);
const int Angle2pos3(double angle);

using namespace serial;

class SCServo : public SCSProtocol {
 public:
  SCServo(void);
  virtual int writeSCS(unsigned char *nDat, int nLen);  //���nLen�ֽ�
  virtual int readSCS(unsigned char *nDat, int nLen);  //����nLen�ֽ�
  virtual int writeSCS(unsigned char bDat);            //���1�ֽ�
  virtual void flushSCS();  //ˢ�½ӿڻ�����
 public:
  // Serial serial;
  Serial ser;
};

#endif
