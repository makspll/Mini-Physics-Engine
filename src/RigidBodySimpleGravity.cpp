#pragma once
#include "MathFunctions.h"
#include "RigidBodySimpleGravity.h"
#include "IForceGenerator.h"


using namespace phy;


RigidBodySimpleGravity::RigidBodySimpleGravity(
	const Vec2d & inGravity)
	:gravity(inGravity) {}

void RigidBodySimpleGravity::updateForce(
	RigidBody	*body, 
	const real  timeDelta)
{
	//if the body is static (has infinite mass) exit
	if (!body->hasFiniteMass()) return;

	//apply the force as force = gravity * mass
	body->addForce(gravity * body->getMass());
}

