/**
 *  main.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 02/07/2015
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "Coop.h"

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "coop_node");
	ros::NodeHandle nh;	
	Coop cooperation(nh);
	cooperation.spin();
	return 0;
}
