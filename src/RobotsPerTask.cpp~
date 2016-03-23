/**
 *  RobotsPerTask.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: **
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "RobotsPerTask.h"

/**
 *
 */
RobotsPerTask::RobotsPerTask(std::vector<Robot> robots, Task task, TaskState state, float progression) :
	task_(task),
	state_(state)
{
	robots_ = robots;
	progression_ = progression;
}

/**
 *
 */
RobotsPerTask::~RobotsPerTask() {
}

/**
 *
 */
bool RobotsPerTask::addRobot(Robot new_robot) {
	for (int i = 0; i < robots_.size(); i++){
		if(new_robot == robots_.at(i)){
			return false;
		}
	}
	robots_.push_back(new_robot);
	return true;	
}

/**
 *
 */
std::vector<Robot> RobotsPerTask::getRobots() {
	return robots_;
}

/**
 *
 */
Task RobotsPerTask::getTask() {
	return task_;
}

/**
 *
 */
TaskState RobotsPerTask::getState() {
	return state_;
}

/**
 *
 */
void RobotsPerTask::setProgression(float progression) {
	progression_ = progression;
}

/**
 *
 */
float RobotsPerTask::getProgression() {
	return progression_;
}
