#pragma once
#include "MathFunctions.h"
#include "RigidBodyFixedSpring.h"

using namespace phy;

RigidBodyFixedSpring::RigidBodyFixedSpring(
	const Vec2d		&attachmentPoint, 
	const Vec2d		&worldAttachmentPoint, 
	const real		springConstant, 
	const real		restLength)
		: attachmentPoint(attachmentPoint), 
		  anchor(worldAttachmentPoint), 
	      k(springConstant), 
       	  restLength(restLength) {}


void RigidBodyFixedSpring::updateForce(
	RigidBody  *affectedBody, 
	const real timeDelta)
{
	//if the body is static (infinite mass) exit
	if (!affectedBody->hasFiniteMass()) return;
	
	//transform the attachment point to world space
	Vec2d localAPoint = affectedBody->pointToWorld(attachmentPoint);

	//calculate the spring vector
	Vec2d spring = anchor - localAPoint;

	//calculate the magnitude of the force
	real springMag = k * (spring.mag() - restLength);

	affectedBody->addForceAtBodyPoint(
		springMag * spring.norm(), attachmentPoint);
}
