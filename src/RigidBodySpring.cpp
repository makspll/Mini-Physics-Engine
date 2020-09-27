#pragma once
#include "MathFunctions.h"
#include "RigidBodySpring.h"

using namespace phy;

RigidBodySpring::RigidBodySpring(
	const Vec2d		&bodyAttachmentPoint,
	RigidBody		*otherBody,
	const Vec2d		&otherBodyAttachmentPoint,
	const real		springConstant,
	const real		restLength)
	:attachmentPoint(bodyAttachmentPoint),
	other(otherBody),
	otherAttachmentPoint(otherBodyAttachmentPoint),
	k(springConstant),
	restLength(restLength) {}


void RigidBodySpring::updateForce(RigidBody * affectedBody, const real timeDelta)
{
	if (!affectedBody->hasFiniteMass()) return;

	//transform the attachment points to world space
	Vec2d localAPoint = affectedBody->pointToWorld(attachmentPoint);
	Vec2d localOtherAPoint = other->pointToWorld(otherAttachmentPoint);

	//calculate spring vector
	Vec2d spring = localOtherAPoint - localAPoint;

	//calculate the magnitude of the force 
	real springMag = k * (spring.mag() - restLength);

	affectedBody->addWorldForceAtLocalPoint(
		springMag * spring.norm(), attachmentPoint);
}
