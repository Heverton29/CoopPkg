/**
 *  RobotNode.h
 *
 *  Version: 1.0.0.0
 *  Created on: 09/08/2015
 *  Modified on: ../08/2015
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef ROBOTNODE_H_
#define ROBOTNODE_H_

#define PI 3.14159

#include <ros/ros.h>
#include <string>
#include <vector>
#include <nav_msgs/Odometry.h>
#include "coop_pkg/Robot.h"
#include "coop_pkg/AddSkill.h"
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include "coop_pkg/SetBusy.h"
#include "Robot.h"


class RobotNode {

public:

	RobotNode(ros::NodeHandle nh);
	RobotNode(ros::NodeHandle nh, Robot robot, std::string ns);
	~RobotNode();

	void spin();
	void spinOnce();
	
	void publishInfoRobot();
	void resetOdometry();

	std::string getNamespace();
	
	void readParameters();
	
private:

	ros::NodeHandle nh_;
	Robot robot_;
	std::string ns_;
	std::string name_;
	std::vector<Skill> skills_;
	int id_;
	bool busy_;
	coop_pkg::Robot info_robot_msg_;
	ros::Publisher info_robot_pub_;
	ros::Subscriber odom_sub_;
	double start_x_, start_y_, start_phi_;
	double curr_x_, curr_y_, curr_phi_, prev_phi_;
	double disp_x_, disp_y_, disp_phi_;	
	bool odom_setted_;
	void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void setRobot(std::string ns, std::string name, int id, bool busy, std::vector<Skill> skills);

	ros::ServiceServer add_skill_srv_;
	ros::ServiceServer set_busy_srv_;
	bool setBusy(coop_pkg::SetBusy::Request &req, coop_pkg::SetBusy::Response &res);
	bool addSkill(coop_pkg::AddSkill::Request &req, coop_pkg::AddSkill::Response &res);
};

#endif /* ROBOTNODE_H_ */
