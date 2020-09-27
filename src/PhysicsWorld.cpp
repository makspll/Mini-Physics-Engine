#include "PhysicsWorld.h"


using namespace phy;

PhysicsWorld::~PhysicsWorld()
{
}


void PhysicsWorld::setUpSimulation()
{
	for (int i = 0; i < container.size(); i++)
	{
		container[i]->calculateDerivedData();
		container[i]->clearForceAccumulators();
	}
}
void PhysicsWorld::stepSimulation(real timeDelta)
{
	forces.updateForces(timeDelta);

	integrateObjects(timeDelta);

	#ifdef DEBUG
		printDebug();
	#endif
}


void PhysicsWorld::integrateObjects(real timeDelta)
{
	for (int i = 0; i < container.size(); i++)
	{
		container[i]->integrate(timeDelta);
	}
}

void PhysicsWorld::addObject(RigidBody* newBody)
{
	container.push_back(newBody);
}

void PhysicsWorld::removeObject(RigidBody* targetBody)
{
	for (int i = 0; i < container.size(); i++)
	{
		if (targetBody == container[i])
		{
			delete container[i];
			container.erase(i + container.begin());
			return;
		}
	}
	return;
}

void PhysicsWorld::printDebug()
{
	for (int i = 0; i < container.size(); i++)
	{
		container[i]->printDebug();
	}
}