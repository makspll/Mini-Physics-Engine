#pragma once
#include "MathFunctions.h"
#include "Polygon.h"
#include "Fixture.h"
#include "Utility.h"
#include "IShape.h"
#include <vector>
#include <iterator>

using namespace phy;

void Polygon::setShape(Vec2d * verts, int count)
{
	//set the vertices amount parameter
	vCount = count;

	for (int i = 0; i < count; i++)
	{
		//index of second vertex wrapped around
		int j = (i + 1) % count;

		//calculate the normal as normalised perpendicular vector
		Vec2d base = verts[i] - verts[j];

		vertices[i] = Vec2d(verts[i]);
		normals[i] = Vec2d(base.y(), base.x(), true);
	}
	calculateCentroid();
}


void Polygon::setSquare(real halfW)
{
	vCount = 4;
	//define default square vertices and normals
	Vec2d square[4] = {
		Vec2d(-halfW ,-halfW),
		Vec2d(halfW ,-halfW),
		Vec2d(halfW , halfW),
		Vec2d(-halfW , halfW) };

	Vec2d sqNormals[4] = {
		Vec2d(0.0f,-1.0f),
		Vec2d(1.0f,0.0f),
		Vec2d(0.0f,1.0f),
		Vec2d(-1.0f,0.0f) };


	//insert them into corresponding tables
	for (int i = 0; i < 4; i++)
	{
		vertices[i] = square[i];
		normals[i] = sqNormals[i];
	}

	centroid = Vec2d(0.0f, 0.0f);
}

std::vector<Vec2d> phy::Polygon::getWorldVertices(const Mat3x3 & transform)
{
	std::vector<Vec2d> list;
	for (int i = 0; i < vCount; i++)
	{
		Vec2d newV = vertices[i];
		list.push_back(newV.toWorld(transform));
	}
	return list;
}

void Polygon::print(std::ostream & where) const
{
	where << "vertices:[";
	for (int i = 0; i < vCount; i++)
	{
		where << vertices[i];
	}
	where << "]\n";

	where << "normals:[";
	for (int i = 0; i < vCount; i++)
	{
		where << normals[i];
	}
	where <<
		"centroid:[" << std::setprecision(3) << centroid << ']'
		<< ']\n';
}

void Polygon::generateAABB(
	AABB         &destAABB, 
	const Mat3x3 &transform)
{
	Vec2d min;
	Vec2d max;

	//find the min and max corners
	for (int i = 0; i < vCount; i++)
	{
		Vec2d transformed = vertices[i].toWorld(transform);
		min[0] = phy::minimum(transformed.x(), min.x());
		min[1] = phy::minimum(transformed.y(), min.y());
		max[0] = phy::maximum(transformed.x(), max.x());
		max[1] = phy::maximum(transformed.y(), max.y());
	}

	//change min and max of the destAABB
	destAABB.set(min, max);
}


void Polygon::calculateMassData(
	MassData	&destMassData,
	real		density)
{
	real inertia = 0.0f;
	real mass = 0.0f;

	//divide the polygon into triangles
	//let rho = density
	//Pn = position of vertex from triangle Origin
	//Pn+1 = next position vertex counter-clockwise
	//In = mass moment of inertia of a single triangle 
	//In = (rho/12) *    
	//     ||(Pn+1 x Pn)|| * 
	//     (Pn+1.y + (Pn+1 dot Pn) + Pn.y)
	//I = sum of In from 1 to No. of vertices

	for (int i = 0; i < vCount; i++)
	{
		//make sure the vertices 'wrap' around
		int j = (i + 1) % vCount;

		//shift the origin to whole polygons centroid
		Vec2d Pn = centroid - vertices[i]; //Pn
		Vec2d P1 = centroid - vertices[j]; //P1

		//area/mass of the individual triangle
		real DoubleTriangleArea = fabs(Pn.Cross(P1));

		//calculate mass from area density
		real triangleMass = density * (DoubleTriangleArea/2.0f);

		mass += triangleMass;

		// calculate inertia of the triangle around for rotation around centroid
		// Izz = Ixx + Iyy = mass * (A.sqrLength() + B.sqrLength() + A.dot(B)) / 6
		// for triangle with sides A B rotated around the centroid of the whole shape
		// source: https://www.codeproject.com/Articles/1215961/Making-a-D-Physics-Engine-Mass-Inertia-and-Forces
		real inertialTriangle = (triangleMass 
								* (Pn.squareMag() + P1.squareMag() + (P1*Pn))) 
									/ 6 ;
		std::cout <<inertialTriangle << '\n';

		//now parallel axis theorem to compensate for shape
		//not being at centroid e.g. I = Io + m*||centroid||
		//inertia += mass * centroid.squareMag(); 
		inertia += inertialTriangle + triangleMass * centroid.mag();
	}

	destMassData.inverseMass = 1.0f / mass;

	if (inertia > 0.0f)
		destMassData.inverseInertia = 1.0f / inertia;
	else
		destMassData.inverseInertia = INFINITY;

	destMassData.centroid = centroid;

}

void Polygon::verticesToWorld(
	Vec2d		 destArray[MAX_VERTS],
	const Mat3x3 &transform) const
{
	for (int i = 0; i < vCount; i++)
	{
		//each vertex is left multiplied by the transform matrix
		//to give the world coordinates of the shape
		Vec2d copy = vertices[i];

		//save these new vertices to destination array
		destArray[i] = copy.toWorld(transform);
	}
}


void Polygon::calculateCentroid()
{
	//centroid position is the average of all x and y values of 
	//its vertices

 	real xPos = 0.0f;
	real yPos = 0.0f;
	for (int i = 0; i < vCount; i++)
	{

		xPos += vertices[i].x();
		yPos += vertices[i].y();
	}

	centroid = Vec2d(xPos, yPos);

	centroid /= vCount;

}



