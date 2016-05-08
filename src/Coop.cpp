/**
 *  Coop.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 02/07/2015
 *  Modified on: 01/04/2016
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "Coop.h"


/**
 *
 */
Coop::Coop(ros::NodeHandle nh) {
	nh_ = nh;
	info_robot_sub_ = nh.subscribe("/robots/info", 20, &Coop::infoRobotCallback, this);
	info_task_sub_ = nh.subscribe("/tasks/info", 10, &Coop::infoTaskCallback, this);
	task_state_pub_ = nh.advertise<coop_pkg::Task>("/tasks/state", 1);
}

/**
 *
 */
Coop::~Coop() {
	nh_.deleteParam("number_of_logged_robots");
	nh_.deleteParam("number_of_registered_tasks");
	info_robot_sub_.shutdown();
	info_task_sub_.shutdown();
	task_state_pub_.shutdown();
}

/**
 *
 */
void Coop::spin() {
	ROS_INFO("Coop Node is up and running!!!");
	ros::Rate loop_rate(10.0);	
	while (ros::ok()) {
		checkLoggedRobots();
		checkIddleRobots();
		checkTasks();
		alocateRobotForAllTasks(robots_, tasks_);
		publishTaskState();
		spinOnce();		
		loop_rate.sleep();
	}
}

/**
 *
 */
void Coop::spinOnce() {
	ros::spinOnce();
}

/**
 *
 */
void Coop::separatePossibleRobots(std::vector<Robot> robots, Task task) {
	competing_robots_.clear();	
	for(int i = 0; i < robots.size(); i++){	
		Robot possible_robot = robots.at(i);		
		if(possible_robot.hasAllSkills(task) && possible_robot.getBusy() == false){
			competing_robots_.push_back(possible_robot);
		}
	}
	for(int i = 0; i<competing_robots_.size(); i++){
		ROS_INFO("Competing Robot: %s", competing_robots_.at(i).getName().c_str());
	}
}

/**
 *
 */
void Coop::alocateRobotForATask(std::vector<Robot> robots, Task task) {
	Robot best_robot;	
	separatePossibleRobots(robots, task);
	for(int i = 0; i < competing_robots_.size(); i++){	
		Robot robot = competing_robots_.at(i);		
		if(competing_robots_.at(i).isTheBest(competing_robots_, task)){
			assignRobots(robot);
			setLocalTaskState(task);
			ROS_INFO("Robot: %s", robot.getName().c_str());
			break;
		}
	}
}

void Coop::setLocalTaskState(Task task) {
	int set_state_position;
	for (int i = 0; i < tasks_.size(); i++) {
		if (task.getId() == tasks_.at(i).getId()) {
			set_state_position = i;
			break;
		}
	}
	tasks_.at(set_state_position).setState(EXECUTING);
}

/**
 *
 */
void Coop::alocateRobotForAllTasks(std::vector<Robot> robots, std::vector<Task> tasks) {
	for(int i = 0; i < tasks.size(); i++){	
		Task task = tasks.at(i);
		if(task.getState() == NOT_ASSIGNED) { 	
			alocateRobotForATask(robots, task);
		}	
	}
}

void Coop::assignRobots(Robot robot) {
	int id_set_busy;
	id_set_busy == 0;
	for (int i = 0; i < robots_.size(); i++) {
		if (robot.getNamespace() == robots_.at(i).getNamespace()) {
			id_set_busy = i;
			break;
		}
	}
	coop_pkg::SetBusy busy_srv;
	busy_srv.request.busy = true;
	if (set_busy_clis_.at(id_set_busy).call(busy_srv)) {
		ROS_INFO("OK: %s!!!", robot.getName().c_str());
	}
}

/**
 *
 */
void Coop::checkLoggedRobots() 
{
	for (int i = 0; i < robots_.size(); i++) 
	{
		//ROS_INFO("%s is logged!!!", robot.getName().c_str());
		if(!robots_.at(i).isStillLogged())
		{
			robots_.erase(robots_.begin() + i);
			set_busy_clis_.at(i).shutdown();
			set_busy_clis_.erase(set_busy_clis_.begin() + i);
			ROS_INFO("%s is not logged anymore!!!", robots_.at(i).getName().c_str());
		}
	}
	int number_of_logged_robots = robots_.size();
	nh_.setParam("/number_of_logged_robots", number_of_logged_robots);
	//ROS_INFO("Number of logged robots: %d", number_of_logged_robots);
}

/**
 *
 */
void Coop::checkIddleRobots() 
{
	if(robots_.size() != 0){
		for (int i = 0; i < robots_.size(); i++) 
		{
			std::cout<<robots_.at(i).getName()<<": "<<robots_.at(i).getBusy()<<"\n";
		}
	}
	int number_of_iddle_robots = 0;
	for (int i = 0; i < robots_.size(); i++) 
	{
		Robot robot(robots_.at(i));
		//ROS_INFO("%s is logged!!!", robot.getName().c_str());
		if(!robot.getBusy())
		{
			number_of_iddle_robots++;
		}
	}
	//ROS_INFO("Number of robots: %d", number_of_iddle_robots);
}

/**
 *
 */
void Coop::checkTasks() 
{
	for (int i = 0; i < tasks_.size(); i++) 
	{
		Task task(tasks_.at(i));
		if (task.getState() != NOT_ASSIGNED) {
			ROS_INFO("%s is not available", task.getName().c_str());
		}
		else {
			ROS_INFO("%s is available", task.getName().c_str());
		}		
		if(task.isExpired())
		{
			tasks_.erase(tasks_.begin() + i);
			ROS_INFO("%s is not available anymore!!!", task.getName().c_str());
		}
	}
	int number_of_registered_tasks = tasks_.size();
	nh_.setParam("/number_of_registered_tasks", number_of_registered_tasks);
	//ROS_INFO("Number of logged robots: %d", number_of_logged_robots);
}

/**
 *
 */
int Coop::getNumberOfCompletedTasks() {
	return number_of_completed_tasks_;
}

/**
 *
 */
bool Coop::addRobot(Robot robot) {
	for (int i = 0; i < robots_.size(); i++){
		if(robot == robots_.at(i)){
			return false;
		}
	}
	robots_.push_back(robot);
	set_busy_clis_.push_back(nh_.serviceClient<coop_pkg::SetBusy>(robot.getNamespace() + "set_busy"));
	return true;	
}

/**
 *
 */
bool Coop::removeRobot(Robot robot) {
	for (int i = 0; i < robots_.size(); i++){
		if(robot == robots_.at(i)){
			robots_.erase(robots_.begin() + i);
			set_busy_clis_.at(i).shutdown();
			set_busy_clis_.erase(set_busy_clis_.begin() + i);
			return true;
		}
	}
	return false;
}

/**
 *
 */
std::vector<Robot> Coop::getRobots() {
	return robots_;
}

/**
 *
 */
bool Coop::addTask(Task task) {
	for (int i = 0; i < tasks_.size(); i++){
		if(task.equals(tasks_.at(i))){
			return false;
		}
	}
	tasks_.push_back(task);
	return true;	
}

/**
 *
 */
bool Coop::removeTask(Task task) {
	for (int i = 0; i < tasks_.size(); i++){
		if(task.equals(tasks_.at(i))){
			tasks_.erase(tasks_.begin()+i);
			return true;
		}
	}
	return false;
}

/**
 *
 */
std::vector<Task> Coop::getTasks() {
	return tasks_;
}

/**
 *
 */
std::vector<RobotsPerTask> Coop::getRobotsPerTasks() {
	return robots_per_tasks_;
}

/**
 *
 */
void Coop::infoRobotCallback(const coop_pkg::Robot::ConstPtr& msg)
{
	Robot robot(msg);
	for (int i = 0; i < robots_.size(); i++) {
		if(robot == robots_.at(i)) {
			robots_.erase(robots_.begin() + i);
			set_busy_clis_.at(i).shutdown();
			set_busy_clis_.erase(set_busy_clis_.begin() + i);
			robots_.push_back(robot);
			set_busy_clis_.push_back(nh_.serviceClient<coop_pkg::SetBusy>(robot.getNamespace() + "set_busy"));		
			return;
		}	
	}
	robots_.push_back(robot);
	set_busy_clis_.push_back(nh_.serviceClient<coop_pkg::SetBusy>(robot.getNamespace() + "set_busy"));
}

/**
 *
 */
void Coop::infoTaskCallback(const coop_pkg::Task::ConstPtr& msg)
{
	Task task(msg);
	for (int i = 0; i < tasks_.size(); i++) {
		if(task == tasks_.at(i)) {
			TaskState state = tasks_.at(i).getState();
			tasks_.erase(tasks_.begin()+i);
			if(task.getState() != ABORTED && task.getState() != FAILED && task.getState() != SUCCEEDED)   {
				task.setState(state);
				tasks_.push_back(task);		
			}		
			return;		
		}	
	}
	tasks_.push_back(task);
}

/**
 *
 */
void Coop::publishTaskState() {
	for (int i = 0; i < tasks_.size(); i++){
		coop_pkg::Task task_state_msg = tasks_.at(i).toMsg();
		task_state_pub_.publish(task_state_msg); 
	}
}
