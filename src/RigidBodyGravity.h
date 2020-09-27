#pragma once
#include "MathFunctions.h"
#include "IForceGenerator.h"

namespace phy {
	class RigidBodyGravity : public IForceGenerator
	{
	public:
		/*
		creates a new gravity force generator with an attractor body
		pointer and gravitational constant. Uses newton's law of 
		gravitation. Can be used to model realistic gravity.
		for two bodies to attract each other, each one needs its 
		own gravity force generator
		f = (Gm1m2)/r
		*/
		RigidBodyGravity(
			RigidBody	*attractor,
			const real  gravityConstant);

		/*
		applies force to the given body over the given time interval
		*/
		void updateForce(
			RigidBody   *attractedBody,
			const real  timeDelta);

	private:
		/*
		the universal constant of gravitation, the realistic value
		is extremely small, in order to see any action use a high value
		*/
		real G;

		/*
		a pointer to the body which will attract the body that the 
		force is applied to
		*/
		RigidBody* attractor;
	};
}