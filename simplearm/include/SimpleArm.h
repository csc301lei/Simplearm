#ifndef SIMPLE_ARM_H
#define SIMPLE_ARM_H

#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"

#include <curses.h>
#include <stdio.h>
#include <stdlib.h>
#include <term.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <vector>

#include "SCServo.cpp"
//#include "Serial.cpp"
#include "math.h"

#define PI 3.141592653589793
#define P_armvel_x 1
#define P_armvel_y 1

class ArmPos
{
public:
  ArmPos() = default;
  double x, y, z, yaw, duration;
};

class SimpleArm
{
public:
  ros::NodeHandle nh;
  ros::Subscriber curPoseSub;
  ros::Subscriber tarPoseSub;
  void callbackCurPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void callbackTarPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void calcJointAngle(ArmPos &armpos);
  void calcJointVel(ArmPos &armpos);
  void ServoPosControl(double theta_1, double theta_2);
  void ServoVelControl(double theta1_vel, double theta2_vel);
  SimpleArm(ros::NodeHandle n);
  SCServo servo;
  ArmPos curpos;
  ArmPos tarpos;
  ~SimpleArm()
  {

    servo.DeltaSyncWritePos(0, 0, 800,
                            4000, 0, 800,
                            0, 0, 0);
  }

  void Update() {}
};

#endif