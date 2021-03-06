/**
 *  Skill.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: **
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "Skill.h"

/**
 *
 */
Skill::Skill (Resource resource, int level) :
	resource_(resource)
{	
	level_ = level;
}

/**
 *
 */
Skill::Skill (const coop_pkg::Skill::ConstPtr& msg) :
	resource_(msg->resource.name, msg->resource.id, msg->resource.description)
{	
	level_ = msg->level;
}

/**
 *
 */
Skill::Skill (coop_pkg::Skill msg) :
	resource_(msg.resource.name, msg.resource.id, msg.resource.description)
{	
	level_ = msg.level;
}

/**
 *
 */
Skill::~Skill() {
}

/**
 *
 */
void Skill::setLevel(int new_level) {
	level_ = new_level;
}

/**
 *
 */
int Skill::getLevel() {
	return level_;
}

/**
 *
 */
Resource Skill::getResource() {
	return resource_;
}

/**
 *
 */
bool Skill::isSufficient(Skill skill) {
	return resource_.equals(skill.resource_) && level_ <= skill.level_;	
}

coop_pkg::Skill Skill::toMsg() {
	coop_pkg::Skill msg;
	msg.resource = resource_.toMsg();
	msg.level = level_;
	return msg;
}

bool Skill::operator==(const Skill& skill)
{
	return resource_ == skill.resource_;
}

bool Skill::operator!=(const Skill& skill) 
{
	return resource_ != skill.resource_;
}
