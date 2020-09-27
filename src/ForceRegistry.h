#pragma once
#include "Precision.h"
#include "RigidBody.h"
#include "IForceGenerator.h"
#include <vector>

namespace phy {
	class ForceRegistry
	{
	public:

		/*
		create a new empty force registry
		*/
		ForceRegistry() {}
		
		/*
		clears and removes a force registry
		*/
		~ForceRegistry();

		/*
		add a force generator, which will apply a force on the
		given body every step of the simulation (acceleration will
		depend on the mass of the object)
		*/
		void addForceGenerator(
			RigidBody		*body, 
			IForceGenerator *fg);

		/*
		removes a single force generator given a pointer to the 
		body and the particular force generator
		*/
		void removeForceGenerator(
			RigidBody		*body, 
			IForceGenerator	*fg);

		/*
		empties the registry by removing all the force generators
		but not the bodies 
		*/
		void clear();

		/*
		apply all forces to the relevant bodies, this should be
		called on every step of the simulation (technically
		the timeDelta is not essential but it is passed in case
		a time-dependant force generator is required in the future)
		*/
		void updateForces(real timeDelta);


	private:

		/*
		holds a single entry tying a body to a force
		*/
		struct Registration
		{
			RigidBody* body;
			IForceGenerator* fg;
			Registration(RigidBody* b, IForceGenerator * f)
				: body(b), fg(f) {}
		};
		
		/*
		the vector holding all entries of force and body pairs
		*/
		std::vector<Registration> registry;
	};
}