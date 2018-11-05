#include "SimpleArm.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simplearm_node");
    ros::NodeHandle n;
    SimpleArm simplearm(n);
    while (ros::ok())
    {
        ArmPos armpos;
        std::cin >> armpos.x;
        std::cin >> armpos.y;
        simplearm.calcJointAngle(armpos);
    }
    ros::spin();
    return 0;
}