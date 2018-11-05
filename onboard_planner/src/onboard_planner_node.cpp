#include <onboard_planner/onboard_planner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "onboard_planner_node");
    ros::NodeHandle n;
    OnboardPlanner glbPlanner(n);

    double start_time = ros::Time::now().toSec();
    while (ros::Time::now().toSec() - start_time < 3)
        continue;
    ROS_INFO("M100 taking off!");
    glbPlanner.startUAV_MonitoredTakeoff();
    ros::spin();
    return 0;
}
