/**
 *  main.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 12/08/2015
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "RobotNode.h"

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "robot_node");
	ros::NodeHandle nh;
	RobotNode robot(nh);
	robot.spin();
	return 0;
}
