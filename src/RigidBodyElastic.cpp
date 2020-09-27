#pragma once
#include "MathFunctions.h"
#include "RigidBodyElastic.h"

using namespace phy;

RigidBodyElastic::RigidBodyElastic(
	const Vec2d		&attachmentPoint, 
	RigidBody		*other, 
	const Vec2d		&oAttachmentPoint, 
	const real		springConstant, 
	const real		restLength)
	:attachmentPoint(attachmentPoint),
	 other(other),
	 otherAttachmentPoint(oAttachmentPoint),
	 k(springConstant),
	 restLength(restLength) {}

void RigidBodyElastic::updateForce(RigidBody *affectedBody, const real timeDelta)
{
	//if the body is static (infinite mass) exit
	if (!affectedBody->hasFiniteMass()) return;

	//get the attachment points in world space
	Vec2d localAPoint = affectedBody->pointToWorld(attachmentPoint);
	Vec2d localOtherAPoint = other->pointToWorld(otherAttachmentPoint);

	//calculate the spring vector
	Vec2d spring = localOtherAPoint - localAPoint;

	//calculate the magnitude of the force
	real springMag = k * (spring.mag() - restLength);

	//do nothing if there is no extension
	if (springMag < 0) return;

	affectedBody->addWorldForceAtLocalPoint(springMag * spring.norm(), attachmentPoint);
}