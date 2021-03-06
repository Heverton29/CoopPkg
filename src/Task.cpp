/**
 *  Task.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 20/05/2015
 *  Modified on: 04/08/2015
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "Task.h"

/**
 *
 */
Task::Task(std::string name, int id, geometry_msgs::Pose2D pose, ros::Time deadline, std::vector<Skill> skills, TaskState state) {
	name_ = name;
	id_ = id;
	pose_ = pose;
	deadline_ = deadline;
	complete_ = isComplete(state);
	state_ = state;
	skills_ = skills;	
}

Task::Task(const coop_pkg::Task::ConstPtr& msg) {
	name_ = msg->name;
	id_ = msg->id;
	pose_ = msg->pose;
	start_ = msg->start;
	end_ = msg->end;
	deadline_ = msg->deadline;
	complete_ = msg->complete;
	state_ = getState(msg->state);
	for(int i = 0; i < msg->skills.size(); i++) {
		Resource resource(msg->skills.at(i).resource.name, msg->skills.at(i).resource.id, msg->skills.at(i).resource.description);
		Skill skill(resource, msg->skills.at(i).level); 		
		skills_.push_back(skill);	
	}	
}

/**
 *
 */
Task::~Task() {
}

TaskState Task::getState(int code) {
	TaskState state;	
	switch (code){
		case 0 : 
			state = NOT_ASSIGNED;
			break;
		case 1 :
			state = WAITING;
			break;
		case 2 :
			state = EXECUTING;
			break;
		case 3 :
			state = ABORTED;
			break;		
		case 4 : 
			state = FAILED;
			break;		
		case 5 :
			state = SUCCEEDED;
			break;
	}
	return state;	
}

/**
 *
 */
std::vector<Skill> Task::getSkills() {
	return skills_;
}

/**
 *
 */
bool Task::isExpired()
{
	return deadline_.toSec() != 0.0 && ros::Time::now() > deadline_;		
}

/**
 *
 */
std::string Task::getName() {
	return name_;
}

/**
 *
 */
int Task::getId() {
	return id_;
}

/**
 *
 */
geometry_msgs::Pose2D Task::getPose() {
	return pose_;
}

/**
 *
 */
ros::Time Task::getDeadline() {
	return deadline_;
}

/**
 *
 */
ros::Time Task::getStart() {
	return start_;
}

/**
 *
 */
ros::Time Task::getEnd() {
	return end_;
}

/**
 *
 */
void Task::setComplete(bool complete) {
	complete_ = complete;
}

/**
 *
 */
bool Task::isComplete() {
	return complete_;
}

/**
 *
 */
bool Task::isComplete(TaskState state) {
	return state == SUCCEEDED;
}


/**
 *
 */
void Task::setState(TaskState state) {
	state_ = state;
}

/**
 *
 */
TaskState Task::getState() {
	return state_;
}

/**
 *
 */
bool Task::addSkill(Skill skill) {
	for (int i = 0; i < skills_.size(); i++){
		Resource resource = skill.getResource();
		if(resource.equals(skills_.at(i).getResource())){
			return false;
		}
	}	
	skills_.push_back(skill);
	return true;
}

/**
 *
 */
bool Task::removeSkill(Skill skill) {
	for (int i = 0; i < skills_.size(); i++){
		Resource resource = skill.getResource();
		if(resource.equals(skills_.at(i).getResource())){
			skills_.erase(skills_.begin()+i);
			return true;
		}
	}
	return false;
}

bool Task::equals(Task task) {
	if(id_ == task.id_ && state_ == task.state_){
		return true;
	}
	return false;
}

coop_pkg::Task Task::toMsg() {
	coop_pkg::Task msg;
	msg.name = name_;
	msg.id = id_;
	msg.pose = pose_;
	msg.start = start_;
	msg.end = end_;
	msg.deadline = deadline_;
	msg.complete = complete_;
	msg.state = state_;
	for(int i = 0; i < skills_.size(); i++) {
		msg.skills.push_back(skills_.at(i).toMsg());
	}
	return msg;
}

bool Task::operator==(const Task& task)
{
	return id_ == task.id_;
}

bool Task::operator!=(const Task& task) 
{
	return id_ != task.id_;
}
