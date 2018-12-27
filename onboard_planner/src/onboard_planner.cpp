#include "onboard_planner/onboard_planner.h"

using namespace std;
using namespace ros;
/*
WayPointsTracker::WayPointsTracker(bool VelocityControl, bool LoopMode, double MaxHorizVel,
                                   double MaxVertVel, double MaxYawRate, double MaxRelHorizDist,
                                   double TolR, double TolYaw, std::vector<WayPoint> AllWayPoints) : velControl(VelocityControl), loopMode(LoopMode), maxHorizVel(MaxHorizVel),
                                                                                                     maxVertVel(MaxVertVel), maxYawRate(MaxYawRate), maxRelHorizDist(MaxRelHorizDist),
                                                                                                     tolR(TolR), tolYaw(TolYaw), allWayPoints(AllWayPoints)
*/
WayPointsTracker::WayPointsTracker(bool VelocityControl, bool LoopMode, double MaxHorizVel,
                                   double MaxVertVel, double MaxYawRate, double MaxRelHorizDist,
                                   double TolR, double TolYaw) : velControl(VelocityControl), loopMode(LoopMode), maxHorizVel(MaxHorizVel),
                                                                 maxVertVel(MaxVertVel), maxYawRate(MaxYawRate), maxRelHorizDist(MaxRelHorizDist),
                                                                 tolR(TolR), tolYaw(TolYaw)
{
    allTracked = false;
    curWayPointId = 0;
    lastWayPointTime = 0.0;
}

bool WayPointsTracker::updateCurWayPoint(WayPoint &curPosTime, WayPoint &deltaPosDur)
{
    /*
    if (allWayPoints.size() == 0)
    {
        // No WayPoint is provided, track (0,0,0,0)
        deltaPosDur.x = -curPosTime.x;
        deltaPosDur.y = -curPosTime.y;
        deltaPosDur.z = -curPosTime.z;
        deltaPosDur.yaw = -curPosTime.yaw;
        deltaPosDur.yaw = deltaPosDur.yaw < -PI_LOCAL ? (deltaPosDur.yaw + 2 * PI_LOCAL) : (deltaPosDur.yaw);
        deltaPosDur.yaw = deltaPosDur.yaw > PI_LOCAL ? (deltaPosDur.yaw - 2 * PI_LOCAL) : (deltaPosDur.yaw);
        deltaPosDur.duration = -curPosTime.duration;
        allTracked = true;

        return false;
    }
    else
    {
        */
    deltaPosDur.x = targetWayPoint.x - curPosTime.x;
    deltaPosDur.y = targetWayPoint.y - curPosTime.y;
    deltaPosDur.z = targetWayPoint.z - curPosTime.z;
    deltaPosDur.yaw = targetWayPoint.yaw - curPosTime.yaw;
    deltaPosDur.yaw = deltaPosDur.yaw < -PI_LOCAL ? (deltaPosDur.yaw + 2 * PI_LOCAL) : (deltaPosDur.yaw);
    deltaPosDur.yaw = deltaPosDur.yaw > PI_LOCAL ? (deltaPosDur.yaw - 2 * PI_LOCAL) : (deltaPosDur.yaw);
    deltaPosDur.duration = targetWayPoint.duration - curPosTime.duration + lastWayPointTime;

    double distPos = sqrt(deltaPosDur.x * deltaPosDur.x + deltaPosDur.y * deltaPosDur.y + deltaPosDur.z * deltaPosDur.z);
    double distYaw = fabs(deltaPosDur.yaw);
    if (distPos < tolR && distYaw < tolYaw && deltaPosDur.duration <= 0)
    {
        //allTracked = curWayPointId == (allWayPoints.size() - 1);
        allTracked = true;
        /*
        if (!allTracked)
        {
            curWayPointId += 1;
            lastWayPointTime = curPosTime.duration;

            return true;
        }
        else if (allTracked && loopMode)
        {
            curWayPointId = 1;
            lastWayPointTime = curPosTime.duration;
            allTracked = false;

            return true;
        }
        else
            return false;
            */
        return true;
    }
    else
        return false;
}

void WayPointsTracker::calcControl(WayPoint &deltaPosDur, Control &curControl)
{
    if (velControl)
    {
        // Control is calculated as v = sgn(x)*min(v_max, |x|^gamma), in which gamma is in (0, 1).
        double distHoriz = sqrt(deltaPosDur.x * deltaPosDur.x + deltaPosDur.y * deltaPosDur.y);
        double tempVertVel = deltaPosDur.z != 0.0 ? fmin(maxVertVel, pow(fabs(deltaPosDur.z), SMOOTHEST_GAMMA)) : 0.0;
        tempVertVel = deltaPosDur.z < 0.0 ? -tempVertVel : tempVertVel;
        double tempYawRate = deltaPosDur.yaw != 0.0 ? fmin(maxYawRate, pow(fabs(deltaPosDur.yaw), SMOOTHEST_GAMMA)) : 0.0;
        tempYawRate = deltaPosDur.yaw < 0.0 ? -tempYawRate : tempYawRate;
        double tempHorizVel = distHoriz != 0.0 ? fmin(maxHorizVel, pow(fabs(distHoriz), SMOOTHEST_GAMMA)) : 0.0;

        double vHoriz;
        if (deltaPosDur.duration > 0.0)
        {
            double optVertVel = deltaPosDur.z / deltaPosDur.duration;
            double optYawRate = deltaPosDur.yaw / deltaPosDur.duration;
            double optHorizVel = distHoriz / deltaPosDur.duration;
            curControl.uz = fabs(optVertVel) < fabs(tempVertVel) ? optVertVel : tempVertVel;
            curControl.uyaw = fabs(optYawRate) < fabs(tempYawRate) ? optYawRate : tempYawRate;
            vHoriz = fabs(optHorizVel) < fabs(tempHorizVel) ? optHorizVel : tempHorizVel;
        }
        else if (allTracked || deltaPosDur.duration <= 0.0)
        {
            curControl.uz = tempVertVel;
            curControl.uyaw = tempYawRate;
            vHoriz = tempHorizVel;
        }

        double relTarYaw = atan2(deltaPosDur.y, deltaPosDur.x) - allWayPoints[curWayPointId].yaw + deltaPosDur.yaw;
        curControl.ux = vHoriz * cos(relTarYaw);
        curControl.uy = vHoriz * sin(relTarYaw);
    }
    else
    {
        double distHoriz = sqrt(deltaPosDur.x * deltaPosDur.x + deltaPosDur.y * deltaPosDur.y);
        double relDist = distHoriz < maxRelHorizDist ? distHoriz : maxRelHorizDist;
        double tempVertVel = deltaPosDur.z != 0.0 ? fmin(maxVertVel, pow(fabs(deltaPosDur.z), SMOOTHEST_GAMMA)) : 0.0;
        tempVertVel = deltaPosDur.z < 0.0 ? -tempVertVel : tempVertVel;
        double tempYawRate = deltaPosDur.yaw != 0.0 ? fmin(maxYawRate, pow(fabs(deltaPosDur.yaw), SMOOTHEST_GAMMA)) : 0.0;
        tempYawRate = deltaPosDur.yaw < 0.0 ? -tempYawRate : tempYawRate;

        if (deltaPosDur.duration > 0.0)
        {
            double optVertVel = deltaPosDur.z / deltaPosDur.duration;
            curControl.uz = fabs(optVertVel) < fabs(tempVertVel) ? optVertVel : tempVertVel;
            double optYawRate = deltaPosDur.yaw / deltaPosDur.duration;
            curControl.uyaw = fabs(optYawRate) < fabs(tempYawRate) ? optYawRate : tempYawRate;
        }
        else if (allTracked || deltaPosDur.duration <= 0.0)
        {
            curControl.uz = tempVertVel;
            curControl.uyaw = tempYawRate;
        }

        double curYaw = atan2(deltaPosDur.y, deltaPosDur.x);
        curControl.ux = relDist * cos(curYaw);
        curControl.uy = relDist * sin(curYaw);
    }
    return;
}

Control WayPointsTracker::input(WayPoint &curPosTime)
{
    WayPoint deltaPosDur;
    Control curControl;
    updateCurWayPoint(curPosTime, deltaPosDur);
    double distPos = sqrt(deltaPosDur.x * deltaPosDur.x + deltaPosDur.y * deltaPosDur.y + deltaPosDur.z * deltaPosDur.z);
    if (distPos < 2)
    {
        calcControl(deltaPosDur, curControl);
    }
    else
    {
        deltaPosDur.x = 0;
        deltaPosDur.y = 0;
        deltaPosDur.z = 0;
        calcControl(deltaPosDur, curControl);
    }
    return curControl;
}

OnboardPlanner::OnboardPlanner(ros::NodeHandle n) : nh(n), nh_("~"), initialized(false), initialize_process_done(false)
{
    string poseTopic;
    /*
    vector<double> wayPointsVec_x;
    vector<double> wayPointsVec_y;
    vector<double> wayPointsVec_z;
    vector<double> wayPointsVec_yaw;
    vector<double> wayPointsVec_duration;
    vector<WayPoint> wayPoints;
    */
    double MaxHorizVel, MaxVertVel, MaxYawRate, MaxRelHorizDist, TolR, TolYaw;
    bool VelocityControl, LoopMode;
    nh_.getParam("PoseTopic", poseTopic);
    //nh_.getParam("WayPointsVec_x", wayPointsVec_x);
    //nh_.getParam("WayPointsVec_y", wayPointsVec_y);
    //nh_.getParam("WayPointsVec_z", wayPointsVec_z);
    //nh_.getParam("WayPointsVec_yaw", wayPointsVec_yaw);
    //nh_.getParam("WayPointsVec_duration", wayPointsVec_duration);
    nh_.getParam("VelocityControl", VelocityControl);
    nh_.getParam("LoopMode", LoopMode);
    nh_.getParam("MaxHorizVel", MaxHorizVel);
    nh_.getParam("MaxVertVel", MaxVertVel);
    nh_.getParam("MaxYawRate", MaxYawRate);
    nh_.getParam("MaxRelHorizDist", MaxRelHorizDist);
    nh_.getParam("TolR", TolR);
    nh_.getParam("TolYaw", TolYaw);
    /*
    for (long unsigned int i = 0; i < wayPointsVec_x.size(); i++)
    {
        WayPoint tempWayPoint;
        tempWayPoint.x = wayPointsVec_x[i];
        tempWayPoint.y = wayPointsVec_y[i];
        tempWayPoint.z = wayPointsVec_z[i];
        tempWayPoint.yaw = wayPointsVec_yaw[i];
        tempWayPoint.duration = wayPointsVec_duration[i];
        wayPoints.push_back(tempWayPoint);
    }
    */
    wayPointsTracker = WayPointsTracker(VelocityControl, LoopMode, MaxHorizVel, MaxVertVel, MaxYawRate, MaxRelHorizDist, TolR, TolYaw);
    glbPoseSub = nh.subscribe("/Robot_1/pose", 3, &OnboardPlanner::callbackGlbPose, this);
    tarPoseSub = nh.subscribe("/Robot_2/pose", 3, &OnboardPlanner::callbackTarPose, this);
    ctrlPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 1);
    sdk_ctrl_authority_service = n.serviceClient<dji_sdk::SDKControlAuthority>("/dji_sdk/sdk_control_authority");
    drone_task_service = n.serviceClient<dji_sdk::DroneTaskControl>("/dji_sdk/drone_task_control");
    takeoffResult = false;
}

void OnboardPlanner::initUAV_VIO()
{
    double start_time = ros::Time::now().toSec();
    double relative_time = 0.0;
    uint8_t flag =
        DJISDK::VERTICAL_VELOCITY |
        DJISDK::HORIZONTAL_VELOCITY |
        DJISDK::YAW_RATE |
        DJISDK::HORIZONTAL_BODY |
        DJISDK::STABLE_ENABLE;

    while (relative_time < 5.0)
    {
        sensor_msgs::Joy control;
        control.axes.push_back(0.0);
        control.axes.push_back(0.0);
        control.axes.push_back(relative_time < 2.7 ? 1.0 : -1.0);
        control.axes.push_back(0.0);
        control.axes.push_back(flag);
        ctrlPub.publish(control);
        ros::spinOnce();
        relative_time = ros::Time::now().toSec() - start_time;
    }
    initialize_process_done = true;
    return;
}

void OnboardPlanner::callbackTarPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    wayPointsTracker.targetWayPoint.x = msg->pose.position.y - 0.5;
    wayPointsTracker.targetWayPoint.y = -msg->pose.position.x;
    wayPointsTracker.targetWayPoint.z = msg->pose.position.z + 0.7;
    wayPointsTracker.targetWayPoint.yaw = 0; //cvtQuat2Yaw(msg->pose.orientation.w,
                                             //            msg->pose.orientation.x,
                                             //            msg->pose.orientation.y,
                                             //            msg->pose.orientation.z);
    wayPointsTracker.targetWayPoint.x -= initialWayPoint.x;
    wayPointsTracker.targetWayPoint.y -= initialWayPoint.y;
    wayPointsTracker.targetWayPoint.z -= initialWayPoint.z;
    //wayPointsTracker.targetWayPoint.yaw -= initialWayPoint.yaw;
    wayPointsTracker.targetWayPoint.duration -= initialWayPoint.duration;
}

void OnboardPlanner::callbackGlbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (takeoffResult)
    {
        WayPoint tempWayPoint;
        tempWayPoint.x = msg->pose.position.y;
        tempWayPoint.y = -msg->pose.position.x;
        tempWayPoint.z = msg->pose.position.z;
        tempWayPoint.yaw = cvtQuat2Yaw(msg->pose.orientation.w,
                                       msg->pose.orientation.x,
                                       msg->pose.orientation.y,
                                       msg->pose.orientation.z);
        tempWayPoint.duration = msg->header.stamp.toSec();
        if (!initialized)
        {
            initialWayPoint = tempWayPoint;
            initialized = true;
        }
        tempWayPoint.x -= initialWayPoint.x;
        tempWayPoint.y -= initialWayPoint.y;
        tempWayPoint.z -= initialWayPoint.z;
        tempWayPoint.yaw -= initialWayPoint.yaw;
        tempWayPoint.duration -= initialWayPoint.duration;

        tempWayPoint.yaw = tempWayPoint.yaw <= -PI_LOCAL ? (tempWayPoint.yaw + 2 * PI_LOCAL) : tempWayPoint.yaw;
        tempWayPoint.yaw = tempWayPoint.yaw > PI_LOCAL ? (tempWayPoint.yaw - 2 * PI_LOCAL) : tempWayPoint.yaw;

        Control curControl = wayPointsTracker.input(tempWayPoint);
        sensor_msgs::Joy control;
        if (wayPointsTracker.velControl)
        {
            uint8_t flag =
                DJISDK::VERTICAL_VELOCITY |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE |
                DJISDK::HORIZONTAL_BODY |
                DJISDK::STABLE_ENABLE;

            control.axes.push_back(curControl.ux);
            control.axes.push_back(curControl.uy);
            control.axes.push_back(curControl.uz);
            control.axes.push_back(curControl.uyaw);
            control.axes.push_back(flag);
            ctrlPub.publish(control);
        }
        else
        {
            uint8_t flag =
                DJISDK::VERTICAL_VELOCITY |
                DJISDK::HORIZONTAL_POSITION |
                DJISDK::YAW_RATE |
                DJISDK::HORIZONTAL_BODY |
                DJISDK::STABLE_ENABLE;

            control.axes.push_back(curControl.ux);
            control.axes.push_back(curControl.uy);
            control.axes.push_back(curControl.uz);
            control.axes.push_back(curControl.uyaw);
            control.axes.push_back(flag);
            ctrlPub.publish(control);
        }
    }
    return;
}

double OnboardPlanner::cvtQuat2Yaw(double q0, double q1, double q2, double q3)
{
    //Euler Angle (3-2-1): Yaw-Pitch-Roll
    double quat[4];
    quat[0] = q0;
    quat[1] = q1;
    quat[2] = q2;
    quat[3] = q3;

    double R_body2NED[3][3] = {(1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3])), 2 * (quat[1] * quat[2] - quat[0] * quat[3]), 2 * (quat[1] * quat[3] + quat[0] * quat[2]),
                               2 * (quat[1] * quat[2] + quat[0] * quat[3]), (1 - 2 * (quat[1] * quat[1] + quat[3] * quat[3])), 2 * (quat[2] * quat[3] - quat[0] * quat[1]),
                               2 * (quat[1] * quat[3] - quat[0] * quat[2]), 2 * (quat[2] * quat[3] + quat[0] * quat[1]), (1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]))};

    double yaw, pitch, roll;
    double pi = PI_LOCAL;
    if (R_body2NED[2][0] != 1 && R_body2NED[2][0] != -1)
    {
        double yaw1, yaw2, pitch1, pitch2, roll1, roll2;
        pitch1 = -asin(R_body2NED[2][0]);
        pitch2 = pi - pitch1;
        double cos_pitch1 = cos(pitch1);
        double cos_pitch2 = cos(pitch2);
        roll1 = atan2(R_body2NED[2][1] / cos_pitch1, R_body2NED[2][2] / cos_pitch1);
        roll2 = atan2(R_body2NED[2][1] / cos_pitch2, R_body2NED[2][2] / cos_pitch2);
        yaw1 = atan2(R_body2NED[1][0] / cos_pitch1, R_body2NED[0][0] / cos_pitch1);
        yaw2 = atan2(R_body2NED[1][0] / cos_pitch2, R_body2NED[0][0] / cos_pitch2);
        if (fabs(pitch1) <= fabs(pitch2))
        {
            yaw = yaw1;
            pitch = pitch1;
            roll = roll1;
        }
        else
        {
            yaw = yaw2;
            pitch = pitch2;
            roll = roll2;
        }
    }
    else if (R_body2NED[2][0] == 1)
    {
        yaw = 0;
        pitch = pi / 2;
        roll = yaw + atan2(R_body2NED[0][1], R_body2NED[0][2]);
    }
    else
    {
        yaw = 0;
        pitch = -pi / 2;
        roll = -yaw + atan2(-R_body2NED[0][1], -R_body2NED[0][2]);
    }

    yaw = yaw <= -pi ? (yaw + 2 * pi) : yaw;
    yaw = yaw > pi ? (yaw - 2 * pi) : yaw;
    pitch = pitch <= -pi ? (pitch + 2 * pi) : pitch;
    pitch = pitch > pi ? (pitch - 2 * pi) : pitch;
    roll = roll <= -pi ? (roll + 2 * pi) : roll;
    roll = roll > pi ? (roll - 2 * pi) : roll;

    return yaw;
}

bool OnboardPlanner::obtainUAVControl()
{
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 1;
    sdk_ctrl_authority_service.call(authority);
    if (!authority.response.result)
    {
        ROS_ERROR("obtain control failed!");
        return false;
    }
    return true;
}

bool OnboardPlanner::startUAV_TakeOff_Land(int task)
{
    dji_sdk::DroneTaskControl droneTaskControl;
    droneTaskControl.request.task = task;
    drone_task_service.call(droneTaskControl);
    if (!droneTaskControl.response.result)
    {
        ROS_ERROR("takeoff_land fail");
        return false;
    }
    return true;
}

void OnboardPlanner::startUAV_MonitoredTakeoff()
{
    if (!obtainUAVControl())
        return;

    ros::Time start_time = ros::Time::now();
    if (!startUAV_TakeOff_Land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
        return;
    ros::Duration(0.01).sleep();
    ros::spinOnce();

    // If M100 is not in the air after 10 seconds, fail.
    while (ros::Time::now() - start_time < ros::Duration(10))
    {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();

    takeoffResult = true;
    return;
}
