#pragma once
#include "MathFunctions.h"
#include "Fixture.h"
#include <iostream>
#include "IShape.h"

namespace phy {
	class RigidBody
	{
	public:
		/*

		*/
		RigidBody();

		/*
		delete and clean up after the body
		*/
		~RigidBody();

		/*
		update position velocity and acceleration
		*/
		void integrate(real timeDelta);

		/*
		attach an existing fixture to this body
		*/
		void attachFixture(Fixture * fixture);

		/*
		add a new fixture to the body
		*/
		void createFixture(IShape * shape, real density);

		/*
		removes a fixture from this body and delete it
		*/
		void removeFixture(Fixture * fixture);

		/*
		recompute the centre of mass, mass and inertia
		*/
		void recalculateMassData();

		/*
		apply a force through the CoM (not instantaneous) force is in world coordinates
		*/
		void addForce(const Vec2d& worldForce);

		/*
		apply a force at a point expressed in world coordinates
		*/
		void addWorldForceAtWorldPoint(const Vec2d & worldForce, const Vec2d & worldPoint);
		
		/*
		apply a force at a point expressed in local coordinates relative to CoM
		*/
		void addWorldForceAtLocalPoint(const Vec2d & worldForce, const Vec2d & localPoint);
		
		/*
		transform a point from local space to world space
		*/
		Vec2d pointToWorld(const Vec2d & localPoint) const;
		/*
		transform a point from world space to local space
		*/
		Vec2d pointToLocal(const Vec2d & worldPoint) const;

		/*
		update the tansform from position and direction, and normalise directions
		*/
		void calculateDerivedData();


		/*
		clear the force buffer (will be zero outside of the integrator)
		*/
		void clearForceAccumulators();

		/*
		
		*/
		void printDebug();

		void setAng(const real inRadians);
		void setAng(const Vec2d & inDir);
		real getAng()const;

		void setMass(const real inMass);
		real getMass() const;

		bool hasFiniteMass() const;
		real getInvMass() const;
		void setInfiniteMass();

		void setPos(const Vec2d & vector);
		void setPos(const real inX,const real inY);
		Vec2d getPos() const;

		void setVel(const Vec2d & vector);
		void setVel(const real inX,const real inY);
		Vec2d getVel() const;

		void setAcc(const real inX,const real inY);
		Vec2d getAcc()const;

		/*
		draw the body to currently initialized allegro display
		*/
		void draw();


	protected:
		void calculateTransform();
		bool isAwake = false;
	private:
		friend std::ostream& operator<< (std::ostream& os, const RigidBody& body);

		/*
		holds world position of the origin, all fixtures vertices
		will be relative to this position
		*/
		Vec2d pos = {0.0f,0.0f};

		/*
		holds the combined centre of mass in local coordinates
		*/
		Vec2d centreOfMass = { 0.0f,0.0f };

		/*
		holds the velocity of the body
		*/
		Vec2d vel = {0.0f,0.0f};

		/*
		holds the acceleration of the body
		*/
		Vec2d acc = {0.0f,0.0f};

		/*
		hold the direction of the body (0 radians points to the right)
		*/
		real ang = 0.0f;

		/*
		holds angular velocity of the body in radians
		*/
		real angVel = 0.0f;

		/*
		holds angular acceleration of the body in radians
		*/
		real angAcc = 0.0f;

		/*
		holds the accumulated forces
		*/
		Vec2d forceAccum = {0.0f,0.0f};

		/*
		torque is a scalar in 2d 
		*/
		real torqueAccum = 0.0f;

		/*
		holds the inertia of the body
		*/
		real inverseInertia = 1.0f;

		/*
		holds the inverse mass of the body
		*/
		real inverseMass = 1.0f;
		
		/*
		holds all the fixtures this body has
		*/
		std::vector<Fixture*> fixtures;

		/*
		damping is required to compensate for rounding inacuracies
		*/
		real linearDamping = 0.01f;

		/*
		
		*/
		real angularDamping = 0.01f;

		/*
		the bodies transform, recalculated every iteration
		*/
		Mat3x3 transform;

	};

	std::ostream& operator<< (std::ostream& os, const RigidBody& body);
}