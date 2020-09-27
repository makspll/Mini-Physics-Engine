#pragma once
#include "MathFunctions.h"
#include "RigidBodyFixedElastic.h"

using namespace phy;

RigidBodyFixedElastic::RigidBodyFixedElastic(
	const Vec2d &attachmentPoint,
	const Vec2d &worldAttachmentPoint,
	const real  springConstant,
	const real  restLength)
:attachmentPoint(attachmentPoint),
anchor(worldAttachmentPoint),
k(springConstant),
restLength(restLength) {}

void RigidBodyFixedElastic::updateForce(
	RigidBody  *affectedBody,
	const real timeDelta)
{
	if (!affectedBody->hasFiniteMass()) return;

	//get points in world space
	Vec2d localAPoint = affectedBody->pointToWorld(attachmentPoint);

	//calculate spring vector
	Vec2d spring = anchor - localAPoint;

	real springMag = k * (spring.mag() - restLength);

	//do nothing if no extension
	if (springMag < 0) return;

	affectedBody->addForceAtBodyPoint(springMag * spring.norm(), attachmentPoint);
}
