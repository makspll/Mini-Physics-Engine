#pragma once
#include "MathFunctions.h"

using namespace phy;


Vec2d Vec2d::toWorld(const Mat3x3& transform)
{
	return transform.transform(*this);
}

Vec2d  Vec2d::toLocal(const Mat3x3& transform)
{
	return transform.getInverse().transform(*this);
}




Mat3x3 Mat3x3::getInverse() const
{
	return Mat3x3(inv(mat));
}

Vec2d Mat3x3::transform(const Vec2d& vector) const
{
	arma::vec3 augmented = arma::vec3({ vector[0],vector[1],1.0f });
	arma::vec3 vec = mat * augmented;
	return Vec2d(vec[0],vec[1]);
}


Vec2d Mat3x3::transformDir(const Vec2d& dir) const
{
	//extract rotation matrix and transform the dir vector
	return Vec2d(mat.submat(0, 1, 1, 1) * dir.vec);
}

Vec2d Mat3x3::transformDirInverse(const Vec2d& dir) const
{
	//extract rotation matrix, invert it and transform the dir vector
	return Vec2d(mat.submat(0, 1, 1, 1).i() * dir.vec);
}

