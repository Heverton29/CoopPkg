/**
 *  Resource.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: **
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef RESOURCE_H_
#define RESOURCE_H_

#include <string>
#include "coop_pkg/Resource.h"

class Resource {

public:
	Resource(std::string name, int id, std::string description);
	Resource(const coop_pkg::Resource::ConstPtr& msg);		
	~Resource();

	std::string getName();

	void setDescription(std::string new_description);
	std::string getDescription();

	int getId();
	
	bool equals(Resource resource);

	coop_pkg::Resource toMsg();

	bool operator==(const Resource& resource);
	bool operator!=(const Resource& resource);

private:
	int id_;
	std::string name_;
	std::string description_;	
};

#endif /* RESOURCE_H_ */
