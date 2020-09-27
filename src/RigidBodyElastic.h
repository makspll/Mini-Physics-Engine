#pragma once
#include "MathFunctions.h"
#include "IForceGenerator.h"

namespace phy {
	class RigidBodyElastic : public IForceGenerator
	{
	public:
		/*
		creates a new elastic force generator with two bodies and
		their respective local attachment points, as well as the
		spring constant and rest length. Uses hooke's law 
		f = -k*x
		*/
		RigidBodyElastic(
			const Vec2d		&attachmentPoint,
			RigidBody		*other,
			const Vec2d		&oAttachmentPoint,
			const real		springConstant,
			const real		restLength);

		/*
		applies force to the given body over the given time interval
		*/
		void updateForce(RigidBody * affectedBody, const real timeDelta);
	private:
		/*
		attachment point on the body (local)
		*/
		Vec2d attachmentPoint;

		/*
		pointer to the other body
		*/
		RigidBody* other;

		/*
		attachment point on the other body (local)
		*/
		Vec2d otherAttachmentPoint;
		
		/*
		the spring constant
		*/
		real k;

		/*
		the length the spring will aim for
		*/
		real restLength;
	};
}
