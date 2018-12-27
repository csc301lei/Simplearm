#include "SimpleArm.cpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simplearm_node");
	ros::NodeHandle n;
	SimpleArm simplearm(n);
	/*
	//double theta_1, theta_2;
	while (ros::ok())
	{
		ArmPos armpos;
		std::cout << "x= ";
		std::cin >> armpos.x;
		std::cout << "y= ";
		std::cin >> armpos.y;
		while (!simplearm.mission_completed)
		{
			simplearm.calcJointAngle(armpos);
			usleep(500000);
		}
		simplearm.mission_completed = false;
	}
*/
	ros::spin();
	return 0;
}