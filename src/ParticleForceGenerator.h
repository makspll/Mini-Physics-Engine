#pragma once
#include "Particle.h"
#include "MathFunctions.h"

namespace phy {
	class ParticleForceGenerator
	{
	public:
		//apply a force to the particle over timeDelta
		virtual void updateForce(Particle *particle, real timeDelta) = 0;
		virtual void printDebug()
		{
			#ifdef _DEBUG
			std::cout << "force generator does not implement printDebug";
			#else
			std::cout << "log not implemented";
			#endif			

		}
	};
};