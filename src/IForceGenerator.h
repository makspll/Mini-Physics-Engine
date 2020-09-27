#pragma once
#include "MathFunctions.h"
#include "RigidBody.h"

namespace phy {
	class IForceGenerator
	{
	public:
		virtual ~IForceGenerator(){};

		//the function that applies force in a specific way to a particle
		virtual void updateForce(RigidBody * body, real timeDelta) = 0;

	};
}