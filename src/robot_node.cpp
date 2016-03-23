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
	/*std::vector<Skill> skills;
	skills.push_back(Skill(Resource(132, "resourc_1", " "), 2));
	skills.push_back(Skill(Resource(452, "resourc_2", " "), 5));
	skills.push_back(Skill(Resource(777, "resourc_3", " "), 4));
	Robot robot_1("robot_1", 1234, false,   skills);*/
	ros::init(argc, argv, "robot_node");
	ros::NodeHandle nh;
	RobotNode robot(nh);
	robot.spin();
	return 0;
}
