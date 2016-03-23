/**
 *  Resource.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: **
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "Resource.h"

/**
 *
 */
Resource::Resource (std::string name, int id, std::string description) {
	name_ = name;
	id_ = id;
	description_ = description;	
}

/**
 *
 */
Resource::Resource (const coop_pkg::Resource::ConstPtr& msg) {
	id_ = msg->id;	
	name_ = msg->name;
	description_ = msg->description;	
}

/**
 *
 */
Resource::~Resource() {
}

/**
 *
 */
void Resource::setDescription(std::string new_description) {
	description_ = new_description;
}

/**
 *
 */
std::string Resource::getName() {
	return name_;
}

/**
 *
 */
std::string Resource::getDescription() {
	return description_;
}

/**
 *
 */
int Resource::getId() {
	return id_;
}

bool Resource::equals(Resource resource) {
	return id_ == resource.id_;
}

coop_pkg::Resource Resource::toMsg() {
	coop_pkg::Resource msg;
	msg.name = name_;
	msg.id = id_;
	msg.description = description_;
	return msg;
}

bool Resource::operator==(const Resource& resource)
{
	return id_ == resource.id_;
}

bool Resource::operator!=(const Resource& resource) 
{
	return id_ != resource.id_;
}
