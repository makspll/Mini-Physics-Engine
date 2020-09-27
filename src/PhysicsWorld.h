#pragma once
#include "MathFunctions.h"
#include <vector>
#include "RigidBody.h"
#include "ForceRegistry.h"

namespace phy {
	class PhysicsWorld
	{
	public:
		~PhysicsWorld();

		/*
		sets up the world with given parameters
		*/
		void setUpSimulation();

		/*
		perform relevant updates and step the simulation by the time step
		*/
		void stepSimulation(real timeDelta);

		/*
		add a new object to the simulation
		*/
		void addObject(RigidBody* newBody);

		/*
		remove an object from the simulation
		*/
		void removeObject(RigidBody* targetBody);

		/*
		print debug information (debug mode only)
		*/
		void printDebug();

		/*
		holds teh 
		*/
		ForceRegistry forces;

	private:

		/*
		perform numerical integration on all objects. Updates all
		the positions and other dynamics data
		*/
		void integrateObjects(real timeDelta);

		/*
		contains all rigid bodies in the simulation
		*/
		std::vector<RigidBody*> container;
		
	};
}