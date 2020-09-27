#include "MathFunctions.h"
#include "RigidBodyFixedGravity.h"


using namespace phy;

RigidBodyFixedGravity::RigidBodyFixedGravity(Vec2d anchor, real gConstant)
	:anchor(anchor), g(gConstant) {}


void RigidBodyFixedGravity::updateForce(RigidBody * affectedBody, real timeDelta)
{
	//if the body is static (infinite mass) exit
	if (!affectedBody->hasFiniteMass()) return;

	Vec2d lengthVector = anchor - affectedBody->getPos();

	affectedBody->addForce(lengthVector.norm() * g);
}