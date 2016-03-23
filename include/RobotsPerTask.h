/**
 *  RobotsPerTask.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: **
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef ROBOTSPERTASK_H_
#define ROBOTSPERTASK_H_

#include <vector>
#include "Robot.h"
#include "Task.h"
#include "TaskState.h"

class RobotsPerTask {

public:

	RobotsPerTask(std::vector<Robot> robots, Task task, TaskState state, float progression = 0);
	~RobotsPerTask();

	bool addRobot(Robot robot);
	std::vector<Robot> getRobots();
	Task getTask();
	TaskState getState();
	void setProgression(float progression);
	float getProgression();

private:
	std::vector<Robot> robots_;
	Task task_;
	TaskState state_;
	float progression_;
};

#endif /* ROBOTSPERTASK_H_ */
