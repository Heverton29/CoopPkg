/**
 *  main.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 12/08/2015
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include <vector>
#include <stdlib.h>     /* srand, rand */
#include <ros/ros.h>
#include "Task.h"
#include "Resource.h"
#include "Skill.h"
#include "coop_pkg/Task.h"

#define MIN_TASK_STATE 0
#define MAX_TASK_STATE 5

int random(int min, int max) {
	return (int) (max - min) * (rand() % 101) / 100 + min;
}

TaskState randomTaskState() {
	TaskState state;
	switch (random(MIN_TASK_STATE, MAX_TASK_STATE)) {
	case 0:
		state = NOT_ASSIGNED;
		break;
	case 1:
		state = WAITING;
		break;
	case 2:
		state = EXECUTING;
		break;
	case 3:
		state = ABORTED;
		break;
	case 4:
		state = FAILED;
		break;
	case 5:
		state = SUCCEEDED;
		break;
	default:
		state = NOT_ASSIGNED;
	}
	return state;
}

int main(int argc, char** argv) {	
	srand(time(NULL));
	ros::init(argc, argv, "task_node");
	ros::NodeHandle nh;
	ros::Publisher task_pub = nh.advertise<coop_pkg::Task>("/tasks/info", 10);
	ros::Rate loop_rate(.2);
	
	std::vector<Task> tasks;

	Resource resource0("Velocidade", 111, "velocidade do robo"); 
	Resource resource1("força", 112, "capacidade do robo de empurrar objetos"); 
	Resource resource2("inteligencia", 113, "capacidade do robo de realizar tarefas interagindo com outros robos"); 
	Resource resource3("visao", 114, "capacidade do robo de enxergar atraves de cameras"); 

	geometry_msgs::Pose2D pose0;
	pose0.x = 1.0;
	pose0.y = 0.3;
	pose0.theta = 0.4; //rads
/*
	geometry_msgs::Pose2D pose1;
	pose0.x = 5.0;
	pose0.y = 0.5;
	pose0.theta = 0.7; //rads

	geometry_msgs::Pose2D pose2;
	pose0.x = 0.0;
	pose0.y = 3.0;
	pose0.theta = 0.6; //rads
*/	
	ros::Duration duration0(100);
	ros::Time deadline0 = ros::Time::now() + duration0;
	
	std::vector<Skill> skills0;
//	std::vector<Skill> skills1;
//	std::vector<Skill> skills2;

	Skill skill00(resource0, 6);
	skills0.push_back(skill00);
	Skill skill01(resource1, 9);
	skills0.push_back(skill01);
/*
	Skill skill10(resource0, 7);
	skills1.push_back(skill10);
	Skill skill11(resource2, 7);
	skills1.push_back(skill11);

	Skill skill20(resource0, 4);
	skills2.push_back(skill20);
	Skill skill21(resource1, 10);
	skills2.push_back(skill21);
	Skill skill22(resource2, 2);
	skills2.push_back(skill22);
*/	
	Task task0("Task0", 211, pose0, deadline0, skills0);
	tasks.push_back(task0);
/*	Task task1("Task1", 212, pose1, deadline0, skills1);
	tasks.push_back(task1);
	Task task2("Task2", 213, pose1, deadline0, skills2);
	tasks.push_back(task2);
*/
	while (nh.ok()) {
		Task task = tasks.at(random(0, tasks.size()));
		task.setState(NOT_ASSIGNED);
		task_pub.publish(task.toMsg());		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
