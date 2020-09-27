#pragma once


namespace phy {
	class IBoundingVolume
	{
		/*
		check if the bounding volume overlaps with another one
		of the same type
		*/
		virtual bool overlaps(const IBoundingVolume &other) = 0;

		/*
		the body approximated by the AABB
	
		RigidBody* body;
			*/
	};
}