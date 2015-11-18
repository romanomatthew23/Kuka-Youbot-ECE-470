//
// Simple demo program that calls the youBot ROS wrapper
//
#include <math.h>
#include <dirent.h>
#include <cstdlib>
#include <iostream>
#include "ros/ros.h"
#include <string>
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"
ros::Publisher platformPublisher;
ros::Publisher armPublisher;
ros::Publisher gripperPublisher;

#include <unistd.h>
#include <termios.h>

char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}


void get_block_locations()
{
	system("rm -rf /home/youbot/libfreenect-inst2/libfreenect/libfreenect/build/bin/dataloc");
	system("timeout 2 /home/youbot/libfreenect-inst2/libfreenect/libfreenect/build/bin/fakenect-record /home/youbot/libfreenect-inst2/libfreenect/libfreenect/build/bin/dataloc > /dev/null");
	DIR *dir;
	struct dirent *ent;
	std::string filename;
	if ((dir = opendir ("/home/youbot/libfreenect-inst2/libfreenect/libfreenect/build/bin/dataloc")) != NULL) {
	  /* print all the files and directories within directory */
	  while ((ent = readdir (dir)) != NULL) {
		if (strstr(ent->d_name,"r")) {	    
			filename = ent->d_name;
			break;
		}
	  }
	  closedir (dir);
	} else {
	  /* could not open directory */
	  perror ("");
	}
	std::cout << filename << std::endl;
	std::string path = "/home/youbot/libfreenect-inst2/libfreenect/libfreenect/build/bin/dataloc/";
	std::string filename_path = path.append(filename); //string of filename included with path
	std::string command = "convert ";
	std::string output_str = " /home/youbot/libfreenect-inst2/libfreenect/libfreenect/build/bin/dataloc/output.png";
	command = (command.append(filename_path).append(output_str));
	const char * c = command.c_str();
	system(c);
}


// create a brics actuator message with the given joint position values
brics_actuator::JointPositions createArmPositionCommand(std::vector<double>& newPositions) {
	int numberOfJoints = 5;
	brics_actuator::JointPositions msg;

	if (newPositions.size() < numberOfJoints)
		return msg; // return empty message if not enough values provided

	for (int i = 0; i < numberOfJoints; i++) {
		// Set all values for one joint, i.e. time, name, value and unit
		brics_actuator::JointValue joint;
		joint.timeStamp = ros::Time::now();
		joint.value = newPositions[i];
		joint.unit = boost::units::to_string(boost::units::si::radian);

		// create joint names: "arm_joint_1" to "arm_joint_5" (for 5 DoF)
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		joint.joint_uri = jointName.str();

		// add joint to message
		msg.positions.push_back(joint);
	}

	return msg;
}

// create a brics actuator message for the gripper using the same position for both fingers
brics_actuator::JointPositions createGripperPositionCommand(double newPosition) {
	brics_actuator::JointPositions msg;

	brics_actuator::JointValue joint;
	joint.timeStamp = ros::Time::now();
	joint.unit = boost::units::to_string(boost::units::si::meter); // = "m"
	joint.value = newPosition;
	joint.joint_uri = "gripper_finger_joint_l";
	msg.positions.push_back(joint);		
	joint.joint_uri = "gripper_finger_joint_r";
	msg.positions.push_back(joint);		

	return msg;
}


// move platform a little bit back- and forward and to the left and right
void movePlatform() {
	geometry_msgs::Twist twist;

	// forward
	twist.linear.x = 0.05;  // with 0.05 m per sec
	twist.linear.y = 0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// backward
	twist.linear.x = -0.05;
	twist.linear.y = -0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// to the left
	twist.linear.x = 0;
	twist.linear.y = 0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// to the right
	twist.linear.y = -0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// stop
	twist.linear.y = 0;
	platformPublisher.publish(twist);
}

// move arm once up and down
void moveArm(double joint_0, double joint_1, double joint_2, double joint_3, double joint_4) {
//void moveArm() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
	joint_0 *= -1.0;
	joint_1 *= -1.0;
	joint_2 *= -1.0;
	joint_3 *= -1.0;
	joint_4 *= -1.0;

	joint_0 += 2.95;
	joint_1 += 1.05 + 3.1415/2.0 - .003;
	joint_2 += -2.44;
	joint_3 += 1.73;
	joint_4 += 2.95;
        //Doubling the moves to see if it works
	// move arm straight up. values were determined empirically
	jointvalues[0] = joint_0;
	jointvalues[1] = joint_1;
	jointvalues[2] = joint_2;
	jointvalues[3] = joint_3;
	jointvalues[4] = joint_4;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);

/*
	// move arm back close to calibration position
	jointvalues[0] = 0.11;
	jointvalues[1] = 0.11;
	jointvalues[2] = -0.11;
	jointvalues[3] = 0.11;
	jointvalues[4] = 0.111;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);

	ros::Duration(2).sleep();
*/

}

// open and close gripper
void moveGripper(int close) {
	brics_actuator::JointPositions msg;
	

	if(close) {
	// close gripper
	msg = createGripperPositionCommand(0);
	gripperPublisher.publish(msg);
	}

	else {
	// open gripper
	msg = createGripperPositionCommand(0.011);
	gripperPublisher.publish(msg);
	}
	//ros::Duration(3).sleep();
	
	
}

bool equ(double a, double b) {
	return abs((a-b)/a) < 0.01;
}
#define d5 217.6
#define a1 33.0
#define d1 147.0
#define a2 155.0
#define a3 135.0
#define PI 3.1415
#define TRAV_SPEED 1.0
void inverse_kin(double * arr, double x,double y,double z,double y2,double z3) {
	double * new_joints = new double(5);
	for (int i = 0; i < 5; i++)
		arr[i] = 0;
	arr[0] = atan2(y,x);
	double z1 = arr[0];
	double o3x = x - d5*cos(z1)*sin(y2);
	double o3y = y - d5*sin(z1)*sin(y2);
	double o3z = z - d5*cos(y2);
	printf("o3: %lf %lf %lf\n",o3x,o3y,o3z);
	//double r = sqrt((o3x - a1*cos(z1))*(o3x - a1*cos(z1)) + (o3y - a1*sin(z1))*(o3y - a1*sin(z1)));
	double r = sqrt(o3x*o3x + o3y*o3y) - a1;	
	double s = o3z - d1;
	double cosq3 = (r*r+s*s - a2*a2 - a3*a3)/(2*a2*a3);
	//case 1
	arr[4] = z3 + PI;
	arr[2] = atan2(sqrt(1 - cosq3*cosq3),cosq3);
	arr[1] = atan2(s,r) - atan2(a3*sin(arr[2]),a2+a3*cos(arr[2]));
	arr[3] = PI/2.0 - y2 - arr[1] - arr[2];
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "youbot_ros_hello_world");
	ros::NodeHandle n;

	platformPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	armPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
	gripperPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);
	sleep(1);

	//movePlatform();
	//moveArm();
	//moveGripper();
//I am going to add in a command terminal interface that controls the robot with the predefined commands

	double jnt_0,jnt_1,jnt_2,jnt_3,jnt_4;
	jnt_0 = jnt_1 = jnt_2 = jnt_3 = jnt_4 = 0.0;
	int play, grip;
	double MOVE_VAL = .01;
	//movePlatform();
	moveArm(jnt_0,jnt_1,jnt_2,jnt_3,jnt_4);
	int cont = 1;
	double new_joints[5];
	double x,y,z;
	double z1, y2, z3;
	z1 = y2 = z3 = 0.0;
	x = y = 0.0;
	z = 550.0;
	while(cont == 1) {
		char comm = getch();
		printf("%c\n",comm);
		switch (comm) {
			case '1':
				jnt_0 += MOVE_VAL;
				break;
			case '2':
				jnt_0 -= MOVE_VAL;
				break;
			case '3':
				jnt_1 += MOVE_VAL;
				break;
			case '4':
				jnt_1 -= MOVE_VAL;
				break;
			case '5':
				jnt_2 += MOVE_VAL;
				break;
			case '6':
				jnt_2 -= MOVE_VAL;
				break;
			case '7':
				jnt_3 += MOVE_VAL;
				break;
			case '8':
				jnt_3 -= MOVE_VAL;
				break;
			case '9':
				jnt_4 += MOVE_VAL;
				break;
			case '0':
				jnt_4 -= MOVE_VAL;
				break;
			case 'c':
				moveGripper(1);
				break;
			case 'o':
				moveGripper(0);
				break;
			case 'r':
				jnt_0 = jnt_1 = jnt_2 = jnt_3 = jnt_4 = 0.0;
				break;
			case 'u':
				jnt_0 = jnt_1 = jnt_2 = jnt_3 = jnt_4 = 0.0;
				jnt_1 = 3.14/2;
				break;
			case 'p':
				printf("claw coordinates:\n");
				printf("X =");
				scanf("%lf",&x);	
				printf("Y =");
				scanf("%lf",&y);
				printf("Z =");
				scanf("%lf",&z);
				printf("ZYZ Euler Angles:\n");
				printf("Z1 determined by claw placement;\n");	
				printf("Y2 =");
				scanf("%lf",&y2);	
				printf("Z3 =");
				scanf("%lf",&z3);
				inverse_kin(new_joints,x,y,z,y2,z3);
				for (int q = 0; q < 5; q++)
					printf("joint[%d] = %lf\n",q,new_joints[q]);
				jnt_0 = new_joints[0];
				jnt_1 = new_joints[1];
				jnt_2 = new_joints[2];
				jnt_3 = new_joints[3];
				jnt_4 = new_joints[4];
				break;
			case 'w':
				x += TRAV_SPEED;
				inverse_kin(new_joints,x,y,z,y2,z3);
				jnt_0 = new_joints[0];
				jnt_1 = new_joints[1];
				jnt_2 = new_joints[2];
				jnt_3 = new_joints[3];
				jnt_4 = new_joints[4];
				break;
			case 'a':
				y += TRAV_SPEED;
				inverse_kin(new_joints,x,y,z,y2,z3);
				jnt_0 = new_joints[0];
				jnt_1 = new_joints[1];
				jnt_2 = new_joints[2];
				jnt_3 = new_joints[3];
				jnt_4 = new_joints[4];
				break;
			case 's':
				x -= TRAV_SPEED;

				inverse_kin(new_joints,x,y,z,y2,z3);
				jnt_0 = new_joints[0];
				jnt_1 = new_joints[1];
				jnt_2 = new_joints[2];
				jnt_3 = new_joints[3];
				jnt_4 = new_joints[4];
				break;
			case 'd':
				y -= TRAV_SPEED;

				inverse_kin(new_joints,x,y,z,y2,z3);
				jnt_0 = new_joints[0];
				jnt_1 = new_joints[1];
				jnt_2 = new_joints[2];
				jnt_3 = new_joints[3];
				jnt_4 = new_joints[4];
				break;
			case 'z':
				z += TRAV_SPEED;

				inverse_kin(new_joints,x,y,z,y2,z3);
				jnt_0 = new_joints[0];
				jnt_1 = new_joints[1];
				jnt_2 = new_joints[2];
				jnt_3 = new_joints[3];
				jnt_4 = new_joints[4];
				break;
			case 'x':
				z -= TRAV_SPEED;

				inverse_kin(new_joints,x,y,z,y2,z3);
				jnt_0 = new_joints[0];
				jnt_1 = new_joints[1];
				jnt_2 = new_joints[2];
				jnt_3 = new_joints[3];
				jnt_4 = new_joints[4];
				break;
			case 'k':
				//k for kamera
				get_block_locations();
				break;
			case '\n':
				break;
			case 'q':
			default:
				std::cout << "exitting";
				cont = 0;
			
				
		}
		moveArm(jnt_0,jnt_1,jnt_2,jnt_3,jnt_4);	
		/*printf("Want to Play? Enter 1: ");
		scanf("%d",&play);
		if(play) {
			printf("Hello. Type in commands to control your fancy Kuka Robot.\nJoint_0 =");
			scanf("%lf",&jnt_0);	
			printf("Joint_1 =");
			scanf("%lf",&jnt_1);	
			printf("Joint_2 =");
			scanf("%lf",&jnt_2);	
			printf("Joint_3 =");
			scanf("%lf",&jnt_3);				
			printf("Joint_4 =");
			scanf("%lf",&jnt_4);
			printf("Gripper (1=closed) =");
			scanf("%d",&grip);	
			moveArm(jnt_0,jnt_1,jnt_2,jnt_3,jnt_4);
			moveGripper(grip);
		}
		else	{
			break;
		}*/
	}
	
	sleep(1);
	ros::shutdown();

	return 0;
}
