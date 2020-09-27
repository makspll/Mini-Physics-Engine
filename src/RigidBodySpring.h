#pragma once
#include "MathFunctions.h"
#include "IForceGenerator.h"

namespace phy {

	class RigidBodySpring : public IForceGenerator
	{
	public:
		/*
		creates a new spring force generator with 2 bodies and 
		corresponding local attachment points as well as the
		spring constant and rest length.
		uses hooke's spring law
		f = -k*x 
		*/
		RigidBodySpring(
			const Vec2d		&bodyAttachmentPoint,
			RigidBody		*otherBody,
			const Vec2d		&otherBodyAttachmentPoint,
			const real		springConstant,
			const real		restLength);

		/*
		applies force to the given body over the given time interval
		*/
		void updateForce(
			RigidBody	*affectedBody,
			const real	timeDelta);

	private:
		/*
		attachment point on the body (local)
		*/
		Vec2d attachmentPoint;

		/*
		a pointer to the other body
		*/
		RigidBody* other;

		/*
		attachment point on the other body (local)
		*/
		Vec2d otherAttachmentPoint;

		/*
		the spring constant of this spring
		*/
		real k;

		/*
		the length at which the spring aims to stay
		*/
		real restLength;
	};

}