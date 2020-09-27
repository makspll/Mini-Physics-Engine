#pragma once
#include "MathFunctions.h"
#include "IForceGenerator.h"



namespace phy{
	class RigidBodySimpleGravity : public IForceGenerator
	{
	public:
		/*
		creates a new simple gravity force generator with a gravity
		vector. Use it to model a force which always acts with the same
		magnitude and direction and depends on the mass of the body
		f = m*g
		*/
		RigidBodySimpleGravity(const Vec2d & inGravity);

		/*
		applies force to the given body over the given time interval
		*/
		void updateForce(RigidBody * body, const real timeDelta);

	private:

		/*
		the force of gravity held by the generator
		*/
		Vec2d gravity;
	};

}