/**
 *  Robot.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 16/03/2015
 *  Modified on: 01/04/2016
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "Robot.h"

Robot::Robot() {
}

/**
 *
 */
Robot::Robot(std::string ns, std::string name, int id, bool busy, std::vector<Skill> skills) {
	ns_ = ns;
	name_ = name;
	id_ = id;
	busy_ = busy;
	skills_ = skills;
}

/**
 *
 */
Robot::Robot(const coop_pkg::Robot::ConstPtr& msg) {
	ns_ = msg->ns;
	name_ = msg->name;
	id_ = msg->id;
	busy_ = msg->busy;
	time_stamp_ = msg->time_stamp;
	//skills_ = msg->skills;
	//pose_ = msg->pose;
	for(int i = 0; i < msg->skills.size(); i++) {
		Resource resource(msg->skills.at(i).resource.name, msg->skills.at(i).resource.id, msg->skills.at(i).resource.description);
		Skill skill(resource, msg->skills.at(i).level); 		
		skills_.push_back(skill);	
	}
}

/**
 *
 */
Robot::~Robot() {
}

bool Robot::isTheBest(std::vector<Robot> robots, Task task) {
	Robot robot_(ns_, name_, id_, busy_, skills_);	
	for(int i = 0; i < robots.size(); i++) {
		Robot robot = robots.at(i);		
		if(robot_ != robot && robot_.calculateFitness(task) < robot.calculateFitness(task)){
			return false;
		}
	}
	return true;
}

/**
 *
 */
bool Robot::hasSkill(Skill skill)
{
	for (int i = 0; i < skills_.size(); i++) {
		if (skills_.at(i) == skill && skills_.at(i).getLevel() != 0){
			return true;			
		}
	}
	return false;
}

/**
 *
 */
bool Robot::hasAtLeastOneSkill(Task task)
{
	std::vector<Skill> skills = task.getSkills();
	for (int i = 0; i < skills.size(); i++) {
		if (hasSkill(skills.at(i))) {
			return true;
		}
	}
	return false;	
}

/**
 *
 */
bool Robot::hasAllSkills(Task task)
{
	Robot robot_(ns_, name_, id_, busy_, skills_);	
	std::vector<Skill> skills = task.getSkills();
	for (int i = 0; i < skills.size(); i++) {
		Skill skill = skills.at(i);		
		if (!robot_.hasSkill(skill)) {
			return false;
		}
	}
	return true;	
}

/**
 *
 */
double Robot::calculateEachFitness(Skill skill)
{	
	Robot robot_(ns_, name_, id_, busy_, skills_);
	double fitness = 0.0;
	for (int i = 0; i < robot_.getSkills().size(); i++) {
		Skill skill_ = robot_.getSkills().at(i);		
		if (skill_ == skill){
			fitness += (skill_.getLevel() / skill.getLevel());		
			break;	
		}
	}
	return fitness;
}

/**
 *
 */
double Robot::calculateFitness(Task task)
{
	Robot robot_(ns_, name_, id_, busy_, skills_);	
	std::vector<Skill> skills = task.getSkills();
	double fitness = 0.0;
	double fit = 0.0;
	for (int i = 0; i < skills.size(); i++) {
		Skill skill = skills.at(i);
		fitness += robot_.calculateEachFitness(skill);
	}
	fitness = fitness / skills.size();
	return fitness;
}

/**
 *
 */
bool Robot::isStillLogged()
{
	return (ros::Time::now() - time_stamp_).toSec() < MAX_SILENCE_DURATION;
}

/**
 *
 */
void Robot::setName(std::string name) {
	name_ = name;
}

/**
 *
 */
std::string Robot::getName() {
	return name_;
}

/**
 *
 */
void Robot::setId(int id) {
	id_ = id;
}

/**
 *
 */
int Robot::getId() {
	return id_;
}

/**
 *
 */
void Robot::setBusy(bool busy) {
	busy_ = busy;
}

/**
 *
 */
bool Robot::getBusy() {
	return busy_;
}

/**
 *
 */
void Robot::setPose(geometry_msgs::Pose2D pose) {
	pose_ = pose;
}

/**
 *
 */
geometry_msgs::Pose2D Robot::getPose() {
	return pose_;
}

/**
 *
 */
void Robot::setSkills(std::vector<Skill> skills) {
	skills_ = skills;
}

/**
 *
 */
std::vector<Skill> Robot::getSkills() {
	return skills_;
}

/**
 *
 */
bool Robot::addSkill(Skill skill) {
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
bool Robot::removeSkill(Skill skill) {
	for (int i = 0; i < skills_.size(); i++){
		Resource resource = skill.getResource();
		if(resource.equals(skills_.at(i).getResource())){
			skills_.erase(skills_.begin()+i);
			return true;
		}
	}
	return false;
}

/**
 *
 */
coop_pkg::Robot Robot::toMsg() {
	coop_pkg::Robot msg;
	msg.ns = ns_;
	msg.name = name_;
	msg.id = id_;
	msg.busy = busy_;
	msg.time_stamp = time_stamp_;
	for( int i = 0; i < skills_.size(); i++) {
		msg.skills.push_back(skills_.at(i).toMsg());
	}
	return msg;
}

bool Robot::operator==(const Robot& robot)
{
	return name_ == robot.name_;
}

bool Robot::operator!=(const Robot& robot) 
{
	return name_ != robot.name_;
}

void Robot::setTimeStamp(ros::Time time_stamp) 
{
	time_stamp_ = time_stamp;
}

/**
 *
 */
ros::Time Robot::getTimeStamp()
{
	return time_stamp_;
}

std::string Robot::getNamespace() {
	return ns_;
}

void Robot::setNamespace(std::string ns) {
	ns_ = ns;
}
