/*
 * SCServo.cpp
 * Ӳ��ͨ�Žӿ�
 * ����: 2016.8.9
 * ����: ̷����
 */

#include "SCServo.h"
#include "string.h"

SCServo::SCServo()
{
  /*
  FILE *output;
  const char script[] = "ls /dev/ttyUSB1";
  const char alarm[] = "ttyUSB";
  char buf[100];
  char *alarm_str;
  output = popen(script, "r");
  fread(buf, sizeof(char), sizeof(buf), output);
  pclose(output);h
  alarm_str = strstr(buf, alarm);

  if (alarm_str != NULL)
  {
          int i = 0;
          while (buf[i])
          {
                  if (buf[i] == 10)
                          buf[i] = '\0';
                  i++;
          }
          if (serial.Open(buf))
          {
                  serial.Set(1000000, 8, 'N', 1);
          }
  }
  else
  {
          printf("Serial Error\n");
  }
  */
  ser.setPort("/dev/ttyUSB1");

  ser.setBaudrate(1000000);

  ser.setParity(parity_none);

  ser.setStopbits(stopbits_one);

  serial::Timeout to = serial::Timeout::simpleTimeout(1000);

  ser.setTimeout(to);

  ser.open();
}

int SCServo::readSCS(unsigned char *nDat, int nLen)
{
  // return serial.Read(nDat, nLen);
  return ser.read(nDat, nLen);
}

int SCServo::writeSCS(unsigned char *nDat, int nLen)
{
  if (nDat == NULL)
  {
    return 0;
  }
  // return serial.Write(nDat, nLen);
  return ser.write(nDat, nLen);
}

int SCServo::writeSCS(unsigned char bDat)
{
  // return serial.Write(&bDat, 1);
  return ser.write(&bDat, 1);
}

void SCServo::flushSCS()
{
  unsigned char temp;
  while (temp != 0)
  {
    // serial.Read(&temp, 1);
    ser.read(&temp, 1);
  }
}

const int Angle2pos1(double angle)
{
  int pos = 11.43 * angle - 0.4841;
  if (pos > 1000)
    return 1000;
  else if (pos < 0)
    return 0;
  else
    return pos;
}

const int Angle2pos2(double angle)
{
  int pos = 11.52 * angle - 17.34;
  if (pos > 1000)
    return 1000;
  else if (pos < 0)
    return 0;
  else
    return pos;
}

const int Angle2pos3(double angle)
{
  int pos = 11.47 * angle - 19.94;
  if (pos > 1000)
    return 1000;
  else if (pos < 0)
    return 0;
  else
    return pos;
}
