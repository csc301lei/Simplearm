#include "SimpleArm.h"

static struct termios initial_settings, new_settings;
static int peek_character = -1;
void init_keyboard();
void close_keyboard();
int kbhit();
int readch();

using namespace std;
using namespace ros;

void init_keyboard()
{
	tcgetattr(0, &initial_settings);
	new_settings = initial_settings;
	new_settings.c_lflag &= ~ICANON;
	new_settings.c_lflag &= ~ECHO;
	new_settings.c_lflag &= ~ISIG;
	new_settings.c_cc[VMIN] = 1;
	new_settings.c_cc[VTIME] = 0;
	tcsetattr(0, TCSANOW, &new_settings);
}

void close_keyboard() { tcsetattr(0, TCSANOW, &initial_settings); }

int kbhit()
{
	char ch;
	int nread;

	if (peek_character != -1)
	{
		return -1;
	}
	new_settings.c_cc[VMIN] = 0;
	tcsetattr(0, TCSANOW, &new_settings);
	nread = read(0, &ch, 1);
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0, TCSANOW, &new_settings);

	if (nread == 1)
	{
		peek_character = ch;
		return 1;
	}
	return 0;
}

int readch()
{
	char ch;
	if (peek_character != -1)
	{
		ch = peek_character;
		peek_character = -1;
		return ch;
	}
	read(0, &ch, 1);
	return ch;
}

// SimpleArm::SimpleArm(ros::NodeHandle n) : nh(n)
SimpleArm::SimpleArm(ros::NodeHandle n)
{
	char act;
	cout << "choose action:" << endl;
	cin >> act;
	if (act == 'a')
	{
		servo.DeltaSyncWritePos(0, 0, 800, 4000, 0, 800, 0, 0, 0);
		cout << "reset done.\n";
	}
	mission_completed = false;

	curPoseSub = nh.subscribe("/Robot_1/pose", 3, &SimpleArm::callbackCurPose, this);
	tarPoseSub = nh.subscribe("/Robot_2/pose", 3, &SimpleArm::callbackTarPose, this);

	//qrPoseSub = nh.subscribe("imgmsg", 3, &SimpleArm::callbackqrPose, this);
}

void SimpleArm::callbackCurPose(
	const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	curpos.x = msg->pose.position.y;
	curpos.y = -msg->pose.position.z;
	curpos.z = -msg->pose.position.x;
}

void SimpleArm::callbackqrPose(const QR_position::Num::ConstPtr &msg)
{
	if (msg->x == 0 && msg->y == 0 && msg->z == 0)
	{
		servo.DeltaSyncWritePos(0, 0, 800, 4000, 0, 800, 0, 0, 0);
		cout << "lost target.\n"
			 << endl;
	}
	else
	{
		tarpos.x = msg->y / 1000 + 0.15;
		tarpos.y = -msg->z / 1000 - 0.05;
		// tarpos.z = -msg->pose.position.x;
		// tarpos.x = tarpos.x - curpos.x;
		// tarpos.y = tarpos.y - curpos.y - 0.205;
		cout << "tarpos.x [based on curpos] = " << tarpos.x << endl;
		cout << "tarpos.y [based on curpos] = " << tarpos.y << endl;

		double dist1 = sqrt(pow(tarpos.x, 2) + pow(tarpos.y, 2));
		double dist2 = fabs(tarpos.z - curpos.z);
		if (dist1 < 0.3)
		{
		}
		else if (dist1 < 0.7 && tarpos.y > 0)
		{
			calcJointAngle(tarpos);
		}
		else
		{
			servo.DeltaSyncWritePos(0, 0, 800, 4000, 0, 800, 0, 0, 0);
			cout << "reset done.\n"
				 << endl;
		}
	}

	sleep(0.5);
}

void SimpleArm::callbackTarPose(
	const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	tarpos.x = msg->pose.position.y;
	tarpos.y = -msg->pose.position.z;
	tarpos.z = -msg->pose.position.x;
	tarpos.x = tarpos.x - curpos.x;
	tarpos.y = tarpos.y - curpos.y - 0.205;
	cout << "tarpos.x [based on curpos] = " << tarpos.x << endl;
	cout << "tarpos.y [based on curpos] = " << tarpos.y << endl;
	double dist1 = sqrt(pow(tarpos.x, 2) + pow(tarpos.y, 2));
	double dist2 = fabs(tarpos.z - curpos.z);
	if (dist1 < 0.3)
	{
	}
	else if (dist1 < 0.8 && tarpos.y > 0 && dist2 < 0.1)
	{
		calcJointAngle(tarpos);
	}
	else
	{
		servo.DeltaSyncWritePos(0, 0, 800, 4000, 0, 800, 0, 0, 0);
		cout << "reset done.\n";
	}
	usleep(500000);
}

void SimpleArm::calcJointAngle(ArmPos &armpos)
{
	double L1 = 0.37, L2 = 0.45;
	// armpos.y = armpos.y - 0.21;
	double theta_1, theta_2;
	// armpos.x = -armpos.x;
	double tempx = -armpos.x;
	double aaa = (pow(tempx, 2) + pow(armpos.y, 2) - pow(L1, 2) - pow(L2, 2)) /
				 (2 * L1 * L2);
	// cout << "acos" << aaa << endl;
	theta_2 = acos(aaa);
	cout << "theta2  = " << theta_2 << endl;
	//解一元二次方程求theta1;
	double a, b, c;
	a = pow((L1 + L2 * cos(theta_2)), 2) + pow(L2 * sin(theta_2), 2);
	// cout << "a = " << a << endl;
	b = -2 * tempx * (L1 + L2 * cos(theta_2));
	// cout << "b = " << b << endl;
	c = pow(tempx, 2) - pow(L2 * sin(theta_2), 2);
	// cout << "c = " << c << endl;
	double d1, d2;
	d1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
	// cout << "d1 = " << d1 << endl;
	d2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
	// cout << "d2 = " << d2 << endl;
	double temp;
	if (d1 > 1 || d1 < -1)
	{
		if (d2 > 1 || d2 < -1)
		{
			cout << "theta1 calculate error!" << endl;
		}
		else
		{
			theta_1 = acos(d2);
			cout << "theta1 = d2 =" << theta_1 << endl;
		}
	}
	else
	{
		if (d2 > 1 || d2 < -1)
		{
			theta_1 = acos(d1);
			cout << "theta1 = d1 =" << theta_1 << endl;
		}
		else
		{
			if (d1 > d2)
			{
				theta_1 = acos(d1);
				cout << "theta1 = d1 =" << theta_1 << endl;
			}
			else
			{
				theta_1 = acos(d2);
				cout << "theta1 = d2 =" << theta_1 << endl;
			}
		}
	}

	theta_1 = (theta_1 * 180 / PI) / 0.088;
	cout << "servo1_target:" << theta_1 << endl;
	theta_2 = 2000 + (theta_2 * 180 / PI) / 0.088;
	cout << "servo2_target:" << theta_2 << endl;

	double theta1_cur, theta2_cur;

	theta1_cur = servo.ReadPos(1);
	theta2_cur = servo.ReadPos(2);
	cout << "theta1_data= " << theta1_cur << "    theta2_data= " << theta2_cur
		 << endl;

	double delta_1, delta_2, x_cur, y_cur, delta_x, delta_y;
	delta_1 = fabs(theta1_cur - theta_1);
	delta_2 = fabs(theta2_cur - theta_2);
	cout << "delta1= " << delta_1 << "   delta2= " << delta_2 << endl;
	theta1_cur = theta1_cur * 0.088 * PI / 180;
	theta2_cur = (theta2_cur - 2000) * 0.088 * PI / 180;
	x_cur = -L1 * cos(theta1_cur) - L2 * cos(theta1_cur + theta2_cur);
	y_cur = L1 * sin(theta1_cur) + L2 * sin(theta1_cur + theta2_cur);
	cout << "x_cur= " << x_cur << "   y_cur= " << y_cur << endl;
	delta_x = fabs(x_cur - tarpos.x);
	delta_y = fabs(y_cur - tarpos.y);
	cout << "deltax= " << delta_x << "   deltay= " << delta_y << endl;
	if (((delta_1 < 5) && (delta_2 < 5)) || ((delta_x < 0.05) && (delta_y < 0.05)))
	{
		mission_completed = true;
		usleep(500000);
		servo.DeltaSyncWritePos(0, 0, 800, 4000, 0, 800, 0, 0, 0);
		sleep(2); //sleep(1);
	}
	else
	{
		// delta_1 = vel_limit(delta_1);
		// delta_2 = vel_limit(delta_2);

		double theta1_vel, theta2_vel;
		theta1_vel = delta_1 * P_armvel_x;
		theta2_vel = delta_2 * P_armvel_y;
		theta1_vel = vel_limit(delta_1);
		theta2_vel = vel_limit(delta_2);
		cout << "theta1_vel" << theta1_vel << endl;
		cout << "theta2_vel" << theta2_vel << endl;
		ServoPosControl(theta_1, theta_2, theta1_vel, theta2_vel);
		// armpos.x = -armpos.x;
		// ArmPos terminal_pos;

		// theta1_cur = (theta1_cur * 0.088) * PI / 180;
		// theta2_cur = ((theta2_cur - 2000) * 0.088) * PI / 180;
		// cout << "theta1_cur= " << theta1_cur * 180 / PI << "    theta2_cur= " <<
		// theta2_cur * 180 / PI << endl;

		/*
terminal_pos.x = L1 * cos(theta1_cur) + L2 * cos(theta1_cur + theta2_cur);
terminal_pos.x = -terminal_pos.x;
terminal_pos.y = L1 * sin(theta1_cur) + L2 * sin(theta1_cur + theta2_cur);
cout << "current x = " << terminal_pos.x << "    current y = " << terminal_pos.y<< endl; 
P_target_pos.x = armpos.x - terminal_pos.x; 
P_target_pos.y = armpos.y -terminal_pos.y; 
cout << "P_TARGET_POS.X= " << P_target_pos.x << "P_TARGET_POS.Y= " << P_target_pos.y << endl; double arm_x_vel, arm_y_vel;
arm_x_vel = P_armvel_x * P_target_pos.x;
arm_y_vel = P_armvel_y * P_target_pos.y;

theta1_vel = L2 * cos(theta1_cur + theta2_cur) * arm_x_vel + L2 * sin(theta1_cur+ theta2_cur) * arm_y_vel; 
theta1_vel = theta1_vel / (L1 * L2 *sin(theta2_cur)); 
theta1_vel = theta1_vel * 180 / PI; 
theta2_vel = (-L1 *cos(theta1_cur) - L2 * cos(theta1_cur + theta2_cur)) * arm_x_vel + (-L1 *sin(theta1_cur) - L2 * sin(theta1_cur + theta2_cur)) * arm_y_vel; 
theta2_vel = theta2_vel / (L1 * L2 * sin(theta2_cur)); 
theta2_vel = theta2_vel * 180 / PI;

*/
	}
}

double SimpleArm::vel_limit(double vel)
{
	if (vel > 900)
	{
		vel = 900;
	}
	if (vel < 250)
	{
		vel = 250;
	}
	return vel;
}
void SimpleArm::ServoPosControl(double theta_1, double theta_2,
								double theta1_vel, double theta2_vel)
{
	servo.DeltaSyncWritePos(theta_1, 0, theta1_vel, theta_2, 0, theta2_vel, 0, 0,
							0);
}

void SimpleArm::calcJointVel(ArmPos &armpos)
{
	double L1 = 0.37, L2 = 0.45;
	// armpos.y = armpos.y - 0.21;
	double theta1_cur, theta2_cur;

	theta1_cur = servo.ReadPos(1);
	theta2_cur = servo.ReadPos(2);
	cout << "theta1_data= " << theta1_cur << "    theta2_data= " << theta2_cur
		 << endl;
	// armpos.x = -armpos.x;
	ArmPos terminal_pos;
	ArmPos P_target_pos;
	theta1_cur = (theta1_cur * 0.088) * PI / 180;
	theta2_cur = ((theta2_cur - 2000) * 0.088) * PI / 180;
	cout << "theta1_cur= " << theta1_cur * 180 / PI
		 << "    theta2_cur= " << theta2_cur * 180 / PI << endl;

	terminal_pos.x = L1 * cos(theta1_cur) + L2 * cos(theta1_cur + theta2_cur);
	terminal_pos.x = -terminal_pos.x;
	terminal_pos.y = L1 * sin(theta1_cur) + L2 * sin(theta1_cur + theta2_cur);
	cout << "current x = " << terminal_pos.x
		 << "    current y = " << terminal_pos.y << endl;
	P_target_pos.x = armpos.x - terminal_pos.x;
	P_target_pos.y = armpos.y - terminal_pos.y;
	cout << "P_TARGET_POS.X= " << P_target_pos.x
		 << "    P_TARGET_POS.Y= " << P_target_pos.y << endl;
	double arm_x_vel, arm_y_vel;
	arm_x_vel = P_armvel_x * P_target_pos.x;
	arm_y_vel = P_armvel_y * P_target_pos.y;
	double theta1_vel, theta2_vel;
	theta1_vel = L2 * cos(theta1_cur + theta2_cur) * arm_x_vel +
				 L2 * sin(theta1_cur + theta2_cur) * arm_y_vel;
	theta1_vel = theta1_vel / (L1 * L2 * sin(theta2_cur));
	theta1_vel = theta1_vel * 180 / PI;
	theta2_vel =
		(-L1 * cos(theta1_cur) - L2 * cos(theta1_cur + theta2_cur)) * arm_x_vel +
		(-L1 * sin(theta1_cur) - L2 * sin(theta1_cur + theta2_cur)) * arm_y_vel;
	theta2_vel = theta2_vel / (L1 * L2 * sin(theta2_cur));
	theta2_vel = theta2_vel * 180 / PI;
	cout << "theta1_vel" << theta1_vel << endl;
	cout << "theta2_vel" << theta2_vel << endl;

	ServoVelControl(theta1_vel, theta2_vel);
}
void SimpleArm::ServoVelControl(double theta1_vel, double theta2_vel)
{
	double veloutValue1, veloutValue2;
	veloutValue1 = fabs(theta1_vel / 0.088);
	cout << "servo1_vel:" << veloutValue1 << endl;
	veloutValue2 = fabs(theta2_vel / 0.088);
	cout << "servo2_vel:" << veloutValue2 << endl;
	if (veloutValue1 > 800)
	{
		veloutValue1 = 800;
	}
	if (veloutValue2 > 800)
	{
		veloutValue2 = 800;
	}
	// servo.WriteSpe(1, outValue1);
	// servo.WriteSpe(2, outValue2);
}

/*
int main(int argc, char **argv)
{
        using namespace std;
        ros::init(argc, argv, "Poscontrol");
        SimpleArm simple_arm;
        SCServo servo;
        ros::Rate loop_rate(50);
        int i = 0;

        while (ros::ok())
        {
                char act;
                cout << "choose action:" << endl;
                cin >> act;
                if (act == 'a')
                {

                        servo.DeltaSyncWritePos(0, 0, 800,
                                                                        4000, 0,
800, 0, 0, 0); cout << "reset done.\n";
                }

                while (act == 'b')
                {
                        int ch = 0;
                        init_keyboard();

                        while (ch != 'q')
                        {
                                i++;
                                servo.DeltaSyncWritePos(250 - 250 * cos(1.5 * i
/ 50), 0, 800, 3650 + 250 * cos(1.5 * i / 50), 0, 800, 0, 0, 0); cout <<
"moving.\n"; if (kbhit())
                                {
                                        ch = readch();
                                }
                                loop_rate.sleep();
                        }
                        close_keyboard();
                        break;
                }
                loop_rate.sleep();
                ros::spinOnce();
        }
        return 0;
}
*/
