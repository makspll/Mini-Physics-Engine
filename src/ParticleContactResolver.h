#pragma once
#include "ParticleContact.h"
#include "MathFunctions.h"

namespace phy {

	class ParticleContactResolver
	{
	public:
		ParticleContactResolver(unsigned iterations);

		//set the maximum iterations
		void setIterations(unsigned iterations);

		//resolve a set of particle contacts for penetration and velocity
		void resolveContats(ParticleContact *contactArray, unsigned numContacts, real timeDelta);
		~ParticleContactResolver();
	protected:
		//maximum iterations allowed
		unsigned int iterations;
		//counter for iterations
		unsigned int iterationsUsed;
	};
}
