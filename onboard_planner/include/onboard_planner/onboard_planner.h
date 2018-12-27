#ifndef ONBOARD_PLANNER_H
#define ONBOARD_PLANNER_H

#include <cmath>
#include <iostream>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/dji_sdk.h>

#define PI_LOCAL 3.141592653589793
#define SMOOTHEST_GAMMA 0.6321205588

/*
 * Units: Every unit follows SI.
 * Coordinate x-y-z: Body frame is F-L-U.
 *                   Global frame is defines as initial F-L-U.
 */

class WayPoint
{
  public:
	WayPoint() = default;
	double x, y, z, yaw, duration;
	// pos and angle are relative to start waypoint
	// while duration is relative to the last waypoint
};

class Control
{
  public:
	// Control in Translation and Yaw, i.e.
	// vx, vy, vz, yawrate when velControl = true
	// x_rel, y_rel, z_rel, yawrate of setpoint when velControl = true
	Control() = default;
	double ux, uy, uz, uyaw;
};

class WayPointsTracker
{
	// WayPointsTracker is designed to calculate control for a group
	// of waypoints. Best velocity/setpoint is chosen according to the given
	// duration. When duration is not long enough, the planner will take position
	// arriving as first priority. If distance between two waypoints is less
	// than radius tolerance, zero velocity/setpoint will be given until duration
	// is ran out. If all waypoints are tracked, the last waypoint will be an
	// attractor.
  public:
	WayPointsTracker() = default;
	/*
	WayPointsTracker(bool VelocityControl, bool LoopMode, double MaxHorizVel,
					 double MaxVertVel, double MaxYawRate, double MaxRelHorizDist,
					 double TolR, double TolYaw,
					 std::vector<WayPoint> AllWayPoints);
					 */
	WayPointsTracker(bool VelocityControl, bool LoopMode, double MaxHorizVel,
					 double MaxVertVel, double MaxYawRate, double MaxRelHorizDist,
					 double TolR, double TolYaw);

	// Switching between VEL and RELTRANS MODE
	bool velControl;
	// Repeat the whole WayPoints
	bool loopMode;

	// Tracking controller Params in VEL MODE
	double maxHorizVel;
	double maxVertVel;
	double maxYawRate;

	// Tracking controller Params in RELTRANS MODE
	double maxRelHorizDist;

	// Radius and angle tolerance for arriving
	double tolR, tolYaw;

	// WatPoints to track
	std::vector<WayPoint> allWayPoints;

	// Temporarily tracked waypoint index
	long unsigned int curWayPointId;
	double lastWayPointTime;
	bool allTracked;

	bool updateCurWayPoint(WayPoint &curPosTime, WayPoint &deltaPosDur);
	void calcControl(WayPoint &deltaPosDur, Control &curControl);
	Control input(WayPoint &curPosTime);
	WayPoint targetWayPoint;
};

class OnboardPlanner
{
  public:
	OnboardPlanner(ros::NodeHandle n);
	WayPointsTracker wayPointsTracker;
	ros::NodeHandle nh, nh_;
	ros::Subscriber glbPoseSub;
	ros::Subscriber tarPoseSub;
	ros::Publisher ctrlPub;
	// void callbackGlbPose(const nav_msgs::Odometry::ConstPtr &msg);
	void callbackGlbPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
	void callbackTarPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
	WayPoint initialWayPoint;
	bool initialize_process_done, initialized;
	double cvtQuat2Yaw(double q0, double q1, double q2, double q3);

	// UAV control
	ros::ServiceClient sdk_ctrl_authority_service;
	ros::ServiceClient drone_task_service;
	bool takeoffResult;
	bool obtainUAVControl();
	bool startUAV_TakeOff_Land(int task);
	void startUAV_MonitoredTakeoff();
	void initUAV_VIO();
};

#endif
