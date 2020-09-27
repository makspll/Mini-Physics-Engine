#pragma once
#include "Particle.h"
#include "MathFunctions.h"

namespace phy {
	class ParticleContact
	{
	public:
		friend class ParticleContactResolver;
		ParticleContact();
		//holds two particles involved in the collision, second particle can be null
		Particle* particle[2];
		//holds the restitution coefficient (1 = perfectly elastic, 0 = perfectly inelastic)
		real restitution;
		//holds the contact normal from the point of view of particle 0
		Vec2d contactNormal;
		//the depth of penetration at the contact
		real penetration; 
	protected:
		//resolves the contact for both velocity and interpenetration
		void resolve(real timeDelta);
		//calculate the separating velocity
		real calculateSeparatingVelocity() const;
	private:
		//handle the impulse calculations for this collision
		void resolveVelocity(real timeDelta);
		//handle the interpenetration for this contact
		void resolveInterpenetration(real timeDelta);

	};
}
