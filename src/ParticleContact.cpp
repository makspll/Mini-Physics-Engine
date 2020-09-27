#include "ParticleContact.h"


using namespace phy;

ParticleContact::ParticleContact()
{
}

void ParticleContact::resolve(real timeDelta)
{
	resolveVelocity(timeDelta);
	resolveInterpenetration(timeDelta);
}

real ParticleContact::calculateSeparatingVelocity() const
{
	Vec2d relativeVelocity = particle[0]->getVel();
	if (particle[1]) relativeVelocity -= particle[1]->getVel();
	return relativeVelocity * contactNormal;
}

void ParticleContact::resolveVelocity(real timeDelta)
{
	//get velocity component in the direction of the contact
	real separatingVelocity = calculateSeparatingVelocity();

	//check if it needs to be resolved
	if (separatingVelocity > 0)
	{
		return;
	}

	//calculate new separating velocity
	real newSepVelocity = -separatingVelocity * restitution;

	//check the velocity build up due to acceleration only
	Vec2d accCausedVelocity = particle[0]->getAcc();
	if (particle[1]) accCausedVelocity -= particle[1]->getAcc();
	real accCausedSepVelocity = accCausedVelocity * contactNormal * timeDelta;

	//if closing velocity is due to acceleration build up, remove it from new sep. velocity
	if (accCausedSepVelocity < 0)
	{
		newSepVelocity += restitution * accCausedSepVelocity;
	}

	real deltaVelocity = newSepVelocity - separatingVelocity;

	//apply the change in velocity to each object in proportion to their inverse mass
	real totalInverseMass = particle[0]->getInvMass();
	if (particle[1]) totalInverseMass += particle[1]->getInvMass();

	//if all particles have infinite mass, impulses are needless
	if (totalInverseMass <= 0) return;

	//calculate magnitude of the impulse to apply
	real impulse = deltaVelocity / totalInverseMass;

	//find the impulse vector or the amount of impulse per unit of inverseMass
	Vec2d impulsePerIMass = contactNormal * impulse;

	//apply impulses: they are applied in direction of the contact, proportionally to inverse mass
	particle[0]->setVel(particle[0]->getVel() + impulsePerIMass * particle[0]->getInvMass());
	if (particle[1])
	{
		particle[1]->setVel(particle[1]->getVel() + impulsePerIMass * -particle[1]->getInvMass());
	}
}

void ParticleContact::resolveInterpenetration(real timeDelta)
{
	if (penetration <= 0) return;

	//the movement will be related to inverse mass of each object
	//so total that
	real totalInverseMass = particle[0]->getInvMass();
	if (particle[1]) totalInverseMass += particle[1]->getInvMass();

	//if all particles have infnite mass do nothing
	if (totalInverseMass <= 0) return;
	
	//find amount of penetration per unif of inverse mass
	Vec2d movePerIMass = contactNormal * (penetration / totalInverseMass);

	//calculate movement amounts
	Vec2d particleMovement2;
	Vec2d particleMovement1 = movePerIMass * particle[0]->getInvMass();

	//apply the resolution
	if(particle[1])
	{
		particleMovement1 = movePerIMass * particle[0]->getInvMass();
	}

	particle[0]->setPos(particle[0]->getPos() + particleMovement1);
	if (particle[1])
	{
		particle[1]->setPos(particle[1]->getPos() + particleMovement2);
	}
}