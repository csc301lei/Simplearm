#include "SimpleArm.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simplearm_node");
    ros::NodeHandle n;
    SimpleArm simplearm(n);
    double theta_1, theta_2;
    /*
    while (ros::ok()) {
      // u8 buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      // simplearm.servo.DeltaSyncWritePos(0, 0, 800,
      //                                  4000, 0, 800,
      //                                  0, 0, 0);
      // simplearm.servo.WritePing(0x01);
      // getchar();
      // printf("Fuck:%x %x %x %x %x %x %x %x\n", buf[0], buf[1], buf[2],
    buf[3],
      // buf[4], buf[5], buf[6], buf[7]);
      theta_1 = simplearm.servo.ReadPos(1);
      theta_2 = simplearm.servo.ReadPos(2);
      cout << "theta1= " << theta_1 << endl;
      cout << "theta2= " << theta_2 << endl;
      usleep(300000);
    }
    */

    while (ros::ok())
    {
        ArmPos armpos;
        std::cin >> armpos.x;
        std::cin >> armpos.y;
        simplearm.calcJointVel(armpos);
    }

    ros::spin();
    return 0;
}