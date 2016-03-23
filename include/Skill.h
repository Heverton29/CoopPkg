/**
 *  Skill.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: **
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SKILL_H_
#define SKILL_H_

#include "Resource.h"
#include "coop_pkg/Skill.h"

class Skill {

public:
	Skill(Resource resource, int level = 0);
	Skill(const coop_pkg::Skill::ConstPtr& msg);
	Skill(coop_pkg::Skill msg);		
	~Skill();
	
	void setLevel(int new_level);
	int getLevel();

	Resource getResource();

	bool isSufficient(Skill skill);

	coop_pkg::Skill toMsg();

	bool operator==(const Skill& skill);
	bool operator!=(const Skill& skill);

private:
	Resource resource_;
	int level_;
};

#endif /* SKILL_H_ */
