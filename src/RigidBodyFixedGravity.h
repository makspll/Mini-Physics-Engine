#pragma once
#include "MathFunctions.h"
#include "IForceGenerator.h"
#include "RigidBodyFixedGravity.h"

namespace phy {

	class RigidBodyFixedGravity : public IForceGenerator
	{
	public:
		/*
		creates a new fixed gravity force generator with an anchor
		position vector and gravitational acceleration constant.
		*/
		RigidBodyFixedGravity(Vec2d anchor, real gConstant);

		void updateForce(RigidBody * affectedBody, real timeDelta);

	private:
		/*
		the constant of gravitational acceleration, 
		on earth 9.8ms-2, 
		*/
		real g;
		/*
		the position of the attracting point
		*/
		Vec2d anchor;
	};

}