#pragma once
#include "MathFunctions.h"
#include "IForceGenerator.h"

namespace phy {
	class RigidBodyFixedElastic : public IForceGenerator
	{
	public:
		/*
		creates a new fixed elastic force generator with a body
		and its local attachment point as well as the spring 
		constant and rest length. uses hooke's law.
		f = -k*x
		*/
		RigidBodyFixedElastic(
			const Vec2d &attachmentPoint,
			const Vec2d &worldAttachmentPoint, 
			const real  springConstant,
			const real  restLength);

		/*
		applies force to the given body over the given time interval
		*/
		void updateForce(
			RigidBody  *affectedBody,
			const real timeDelta);
	private:
		/*
		attachment point on the body (local)
		*/
		Vec2d attachmentPoint;

		/*
		the spring attachment point (world)
		*/
		Vec2d anchor;

		/*
		the spring constant
		*/
		real k;

		/*
		the length at which the spring aims to be
		*/
		real restLength;
	};
}