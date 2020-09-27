#pragma once
#include "MathFunctions.h"
#include "RigidBodyGravity.h"

using namespace phy;

RigidBodyGravity::RigidBodyGravity(RigidBody* attractor, const real gravityConstant)
	: attractor(attractor), G(gravityConstant) {}


void RigidBodyGravity::updateForce(
	RigidBody  *body,
	const real timeDelta)
{
	//if the body is static (infinite mass) exit
	if (!body->hasFiniteMass()) return;

	Vec2d toAttractor = attractor->getPos() - body->getPos();
	Vec2d direction = toAttractor.norm();

	real productMass = 1 /
		(body->getInvMass() * attractor->getInvMass());

	//apply newton's law of gravitation
	body->addForce(direction *
		((G * productMass) /
			toAttractor.squareMag()));
}
