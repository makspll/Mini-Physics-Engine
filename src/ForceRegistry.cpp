#pragma once
#include "ForceRegistry.h"


using namespace phy;

ForceRegistry::~ForceRegistry()
{
	//make sure there is no memory leak due to force generators
	//not being deleted, pointers will clear as they go out of scope
	clear();
}


void ForceRegistry::addForceGenerator(
	RigidBody		*body, 
	IForceGenerator	*fg)
{
	registry.push_back(Registration(body, fg));
}

void ForceRegistry::removeForceGenerator(
	RigidBody		*body, 
	IForceGenerator *fg)
{
	for (int i = 0; i < registry.size(); i++)
	{
		if (registry[i].body == body)
			if (registry[i].fg == fg) 
				registry.erase(i + registry.begin());
	}
}

void ForceRegistry::clear()
{
	for (int i = 0; i < registry.size(); i++)
	{
		registry.erase(i + registry.begin());
	}
}

void ForceRegistry::updateForces(real timeDelta)
{
	for (int i = 0; i < registry.size(); i++)
	{
		registry[i].fg->updateForce(registry[i].body, timeDelta);
	}
}