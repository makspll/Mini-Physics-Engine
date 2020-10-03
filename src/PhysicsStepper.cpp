#pragma once
#include "MathFunctions.h"
#include <vector>
#include "ParticlePhysicsWorld.h"
#include "PhysicsWorld.h"
#include "PhysicsStepper.h"

using namespace phy;

PhysicsStepper::PhysicsStepper(PhysicsWorld* pw,ParticlePhysicsWorld* ppw){
    this->pw = pw;
    this->ppw = ppw;
    last_frame_time = std::chrono::high_resolution_clock::now();
};

void PhysicsStepper::stepSimulation(){
    // timing settings
	auto curr_frame_time = std::chrono::high_resolution_clock::now();

    // calculate frame duration
    std::chrono::duration<real> duration = curr_frame_time - last_frame_time;

    // step simulation by the frame duration 
    pw->stepSimulation(std::chrono::duration_cast<std::chrono::duration<double>>(duration).count());
    ppw->stepSimulation(std::chrono::duration_cast<std::chrono::duration<double>>(duration).count());
    
    // update last frame time
    last_frame_time = curr_frame_time;
}
