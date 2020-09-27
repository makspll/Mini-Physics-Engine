#include "ParticleContactResolver.h"

using namespace phy;

ParticleContactResolver::ParticleContactResolver(unsigned inIterations)
{
	iterations = inIterations;
}


ParticleContactResolver::~ParticleContactResolver()
{
}

void ParticleContactResolver::resolveContats(ParticleContact *contactArray, unsigned numContacts, real timeDelta)
{
	unsigned i;

	iterationsUsed = 0;
	while (iterationsUsed < iterations)
	{
		//find the contact with largest closing velocity
		real max = REAL_MAX;
		unsigned maxIndex = numContacts;
		for (i = 0; i < numContacts; i++)
		{
			real sepVel = contactArray[i].calculateSeparatingVelocity();
			if (sepVel < max &&
				(sepVel < 0 || contactArray[i].penetration < 0))
			{
				max = sepVel;
				maxIndex = i;
			}
		}
		//do we have anything worth resolving ?
		if (maxIndex == numContacts) break;

		//resolve this contact
		contactArray[maxIndex].resolve(timeDelta);
		iterationsUsed++;
	}
}