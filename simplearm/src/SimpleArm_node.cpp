#include "SimpleArm.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simplearm_node");
    ros::NodeHandle n;
    SimpleArm simplearm(n);
    double theta_1, theta_2;
    while (ros::ok())
    {
        theta_1 = simplearm.servo.ReadPos(1);
        theta_2 = simplearm.servo.ReadPos(2);
        cout << "theta1_cur= " << theta_1 << endl;
        cout << "theta2_cur= " << theta_2 << endl;
        sleep(1);
    } /*
    while (ros::ok())
    {
        ArmPos armpos;
        std::cin >> armpos.x;
        std::cin >> armpos.y;
        simplearm.calcJointVel(armpos);
    }
    */

    ros::spin();
    return 0;
}