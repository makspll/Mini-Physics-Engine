#pragma once
#include "MathFunctions.h"
#include <vector>
#include "ParticlePhysicsWorld.h"
#include "PhysicsWorld.h"


namespace phy {
    // The class which deals with stepping the simulation and frame timings
	class PhysicsStepper
	{
    public:
        PhysicsStepper(PhysicsWorld* pw,ParticlePhysicsWorld* ppw);
        void stepSimulation();


    private:
        ParticlePhysicsWorld* ppw;
        PhysicsWorld* pw;
        std::chrono::_V2::system_clock::time_point last_frame_time;
    };

}