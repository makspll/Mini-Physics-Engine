#pragma once
#include "Particle.h"
#include "MathFunctions.h"
#include "ParticleForceGenerator.h"
#include <vector>

namespace phy {
	class ParticleForceRegistry final
	{
	public:
		//add a particle-force generator pair to the register
		void add(Particle* particle, ParticleForceGenerator* fg);

		//remove a particle-force generator pair from the register
		//if corresponding pair is not found, nothing is done
		void remove(Particle* particle, ParticleForceGenerator* fg);

		//clear the register
		void clear();

		//call all force generators to apply forces to their particles
		void updateForces(real timeDelta);

		//print debug information
		void printDebug();
		ParticleForceRegistry();
		~ParticleForceRegistry();

	protected:
		/*
			holds a single entry corresponding to a particle and force
			generator
		*/
		struct ParticleForceRegistration
		{
			Particle* particle;
			ParticleForceGenerator* fg;
			void printDebug();
			ParticleForceRegistration(Particle* particle, ParticleForceGenerator* fg);
		};

		
		typedef std::vector<ParticleForceRegistration> Registry;
		// the list of all force generator particle pairs
		Registry registrations;
	};
	

}
