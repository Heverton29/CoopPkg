/**
 *  Task.h
 *
 *  Version: 1.0.0.0
 *  Created on: 20/05/2015
 *  Modified on: 04/08/2015
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_H_
#define TASK_H_

#include <ros/ros.h>
#include "coop_pkg/Task.h"
#include <vector>
#include <string>
#include <geometry_msgs/Pose2D.h>
#include "Skill.h"
#include "TaskState.h"

class Task {

public:

	Task(std::string name, int id, geometry_msgs::Pose2D pose, ros::Time deadline, std::vector<Skill> skills, TaskState state = NOT_ASSIGNED);

	Task(const coop_pkg::Task::ConstPtr& msg);
	~Task();

	void spin();
	void spinOnce();

	std::string getName();

	int getId();

	geometry_msgs::Pose2D getPose();

	ros::Time getStart();
	ros::Time getEnd();

	void setComplete(bool complete);
	bool isComplete();
	void setState(TaskState state);
	TaskState getState();

	bool addSkill(Skill new_skill);
	bool removeSkill(Skill skill);
	std::vector<Skill> getSkills();

	bool equals(Task task);

	coop_pkg::Task toMsg();

	ros::Time getDeadline();
	bool isExpired();

	bool operator==(const Task& task);
	bool operator!=(const Task& task);

private:

	std::string name_;
	int id_;
	geometry_msgs::Pose2D pose_;
	ros::Time start_;
	ros::Time end_;
	ros::Time deadline_;
	bool complete_;
	TaskState state_;
	std::vector<Skill> skills_;

	TaskState getState(int code);
	bool isComplete(TaskState state);
};

#endif /* TASK_H_ */
