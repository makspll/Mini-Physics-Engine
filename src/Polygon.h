#pragma once
#include "MathFunctions.h"
#include "IShape.h"
#include <vector>
#include <iomanip>

namespace phy {

	/*
	describes a region of space enclosed between an arbitrary 
	number of vertices (at least 3)
	*/
	class Polygon : public IShape
	{

	public:
		/*
		sets the polygon's shape from the given vertices
		*/
		void setShape(Vec2d * points, int count);

		/*
		assign the shape to a sized square with 0,0 centroid
		*/
		void setSquare(real halfWidth);

		/*
		return list of vertices after transforming them with
		given transform TEMPORARY DEBUG FUNCTION
		*/
		std::vector<Vec2d> getWorldVertices(const Mat3x3 & transform);
		
		/*
		prints to stream
		*/
		void print(std::ostream& where) const;


	protected:
		/*
		calculate a tight fitting AABB bounding volume and write
		it to destAABB given a transform of the shape
		*/
		void generateAABB(
			AABB		 &destAABB,
			const Mat3x3 &transform);

		/*

		*/
		void calculateMassData(
			MassData	&destMassData,
			real		density);

		/*
		writes a new array with vertices transformed to world space
		*/
		void verticesToWorld(
			Vec2d		 destArray[MAX_VERTS],
			const Mat3x3 &transform) const;

		/*
		find new centroid from vertices
		*/
		void calculateCentroid();


	private:
		friend class Fixture;
		/*
		the list of all vertices in local coordinates, in 
		counter-clockwise order
		(with respect to the body that owns this shape)
		*/
		Vec2d vertices[MAX_VERTS];

		/*
		the list of all normals, all must be normalised,
		counter-clockwise order
		(world)
		*/
		Vec2d normals[MAX_VERTS];

		/*
		the number of actual vertices and normals of the polygon
		*/
		int vCount = 0;

		/*
		the geometric center of the shape and its centre of mass!
		NOT AN OFFSET POSITION
		ALL VERTICES ARE "BAKED IN" TO CREATE A SHAPE THAT'S 
		OFFSET, CHANGE ITS VERTICES
		*/
		Vec2d centroid = {0,0};

	};



}