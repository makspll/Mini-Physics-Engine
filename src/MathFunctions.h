#pragma once
#include <armadillo>
#include "Precision.h"
#include <assert.h>
#include <math.h>
#include <iostream>
#include <iomanip>

namespace phy {
	class Mat3x3;

	class Vec2d {
	public:
		friend class Mat3x3;
		friend std::ostream& operator<<(std::ostream& os, const Vec2d& vec);
		/*
		creates a new 2d vector, by default will be (0,0)
		*/
		inline Vec2d()
		{
			vec = { 0,0 };
		}

		/*

		*/
		inline Vec2d(real x, real y, bool normalise = false)
		{
			vec = { x,y };
			if (normalise) vec = arma::normalise(vec);
		}

		/*

		*/
		inline Vec2d(Avec2d vector)
		{
			vec = vector;
		}

		/*
		construct a direction vector from an angle IN RADIANS
		*/
		explicit inline Vec2d(real radians)
		{
			vec = Avec2d{cos(radians),sin(radians) };
		}

		/*
		get x component
		*/
		inline real x() const
		{
			return vec[0];
		}

		/*
		get y component
		*/
		inline real y() const
		{
			return vec[1];
		}
		/*
		return transformed vector from local to world coordinates 
		*/
		Vec2d toWorld(const Mat3x3& transform);
		
		/*
		return transformed vector from world to local coordinates
		*/
		Vec2d toLocal(const Mat3x3& transform);

		/*
		perform 2d cross product (MATHS HACK)
		will give area of parallelogram defined by vectors
		*/
		real Cross(const Vec2d& other) const
		{
			return (vec[0] * other.vec[1]) - (vec[1] * other.vec[0]);
		}

		/*
		performs linear interpolation on a vector
		to get a new shorter or longer one
		 for example 
		lerp [1,1] with ratio of 0.5 will result in [0.5,0.5]
		*/
		inline Vec2d lerp(const Vec2d& vec,real ratio)
		{
			return Vec2d (ratio * vec.vec);
		}

		/*
		returns magnitude of vec2d vector in meters
		*/
		inline real mag() const
		{
			return sqrt(arma::sum(vec % vec));
		}

		/*
		returns the square of the magnitude of vec2d vector in meters
		*/
		inline real squareMag()
		{
			return arma::sum(vec % vec);
		} const

		/*
		return normalized vector
		*/
		inline Vec2d norm()
		{
			return Vec2d(normalise(vec));
		} const

		/*
		add a vector multiplied by a scalar first
		*/
		inline void addScaledVec(const Vec2d& vector,const real scalar)
		{
			vec += (vector.vec * scalar);
		}

		/*
		will turn the two vectors into an orthonormal basis, a's direction won't change
		*/
		inline static void makeOrthonormalBasis(Vec2d* a, Vec2d* b)
		{
			a->norm();
			real tempx = b->vec[0];
			b->vec[0] = b->vec[1];
			b->vec[1] = -tempx;
			b->norm();
		}

		/*
		set all components to zero
		*/
		inline void zeros()
		{
			vec.zeros();
		}

		/*
		scalar product
		*/
		inline Vec2d operator* (const real value) const
		{
			return Vec2d(vec * value);
		}

		/*
		scalar division
		*/
		inline Vec2d operator/ (const real value) const
		{
			return Vec2d(vec / value);
		}

		/*
		scalar addition
		*/
		inline Vec2d operator+ (const real value) const
		{
			return Vec2d(vec + value);
		}

		/*
		scalar difference
		*/
		inline Vec2d operator- (const real value) const
		{
			return Vec2d(vec - value);
		}

		/*
		dot product
		*/
		inline real operator* (const Vec2d& vector) const
		{
			return dot(vec, vector.vec); 
		}

		/*
		element wise vector division
		*/
		inline Vec2d operator/ (const Vec2d& vector) const
		{
			return Vec2d(vec / vector.vec);
		}

		/*
		component product (element wise product)
		*/
		inline Vec2d operator% (const Vec2d& vector) const
		{
			return Vec2d(vec % vector.vec);
		}

		/*
		vector product
		*/
		inline Vec2d operator+ (const Vec2d& vector) const
		{
			return Vec2d(vec + vector.vec);
		}

		/*
		vector difference
		*/
		inline Vec2d operator- (const Vec2d& vector) const
		{
			return Vec2d(vec - vector.vec);
		}

		/*
		scalar product
		*/
		inline void operator+= (const real value)
		{
			vec += value;
		}

		/*
		scalar difference
		*/
		inline void operator-= (const real value)
		{
			vec -= value;
		}

		/*
		scalar product
		*/
		inline void operator*= (const real value)
		{
			vec *= value;
		}

		/*
		scalar division
		*/
		inline void operator/= (const real value)
		{
			vec /= value;
		}

		/*
		vector product
		*/
		inline void operator+= (const Vec2d& vector)
		{
			vec += vector.vec;
		}

		/*
		vector difference
		*/
		inline void operator-= (const Vec2d& vector)
		{
			vec -= vector.vec;
		}

		/*
		vector component wise product
		*/
		inline void operator%= (const Vec2d& vector)
		{
			vec %= vector.vec;
		}

		/*
		vector component wise division
		*/
		inline void operator/= (const Vec2d& vector)
		{
			vec /= vector.vec;
		}

		/*
		negation
		*/
		inline Vec2d operator-(void) const
		{
			return Vec2d(-vec);
		} 

		/*
		access operators
		*/
		inline real& operator[] (const int index)
		{
			assert(index == 0 || index == 1);
			return vec[index];
		}

		/*
		access operators
		*/
		inline real operator[] (const int index) const
		{
			assert(index == 0 || index == 1);
			return vec[index];
		}

		private:

		/*
		the internal vector representation
		*/
		Avec2d vec;
	};


	inline std::ostream& operator<<(std::ostream& os, const Vec2d& vec)
	{
		os << '[' << std::setprecision(3) << vec.x() << ',' <<
			std::setprecision(3) << vec.y() << ']';
		return os;
	}

	class Mat3x3
	{
	public:
		/*
		
		*/
		Mat3x3() {}
		/*
		get inverse of this matrix
		*/
		Mat3x3 getInverse() const;

		/*
		left multiply the matrix by vector
		*/
		Vec2d transform(const Vec2d& vector) const;

		/*
		transform direction vector
		*/
		Vec2d transformDir(const Vec2d& dir) const;

		/*
		transform direction vector by inverse rotation
		*/
		Vec2d transformDirInverse(const Vec2d& dir) const;

		/*
		create new 3x3 matrix from Armadillo's matrix
		*/
		inline Mat3x3(Amat3x3 matrix)
		{
			mat = matrix;
			assert(mat.row(3)[0] == mat.row(3)[1] == 0 && mat.row(3)[3] == 1);
		}

		/*
		left multiply the matrix by vector as an affine transformation
		*/
		inline Vec2d operator* (const Vec2d& vector) const
		{
			return Vec2d(affmul(mat, vector.vec));
		}

		/*
		left multiply the matrix by matrix
		*/
		inline Mat3x3 operator* (const Mat3x3& matrix) const
		{
			return Mat3x3(mat * matrix.mat);
		}

		/*
		invert this matrix
		*/
		inline void invert()
		{
			mat = inv(mat);
			assert(mat.row(3)[0] == mat.row(3)[1] == 0 && mat.row(3)[3] == 1);
		}

		/*
		get the transpose of this matrix
		*/
		inline Mat3x3 getTranspose() const
		{
			return Mat3x3(trans(mat));
		}

		/*
		transpose this matrix
		*/
		inline void toTranspose()
		{
			mat = mat.t();
		}
		/*
		set this matrix to be the inverse of the other matrix
		*/
		inline void setInverse(const Mat3x3& other)
		{
			mat = other.mat.i();
		}

		/*
		get determinant of matrix
		*/
		inline real det() const
		{
			return arma::det(mat);
		}

		/*
		set this matrix to be the rotation matrix corresponding to given angle + translation
		e.g. if you transform a vector with a resulting matrix, it will be rotated to
		with this orientation and translated to this new pos
		*/
		inline void setDirAndPos(const real ang, const Vec2d& pos)

		{
			//derive rotation matrix from angle, first calculate direction vector then assign rotation matrix
			//[x,y] = [x -y] = [cos ang, sin ang]
			//		  [y  x]
			// however the transformation is affine, so bottom row is always 0,0,1
			real x = cos(ang);
			real y = sin(ang);

			mat(0, 0) = x;
			mat(0, 1) = -y;
			mat(1, 0) = y;
			mat(1, 1) = x;
			//set bottom row of rotation part to zeros
			mat(2, 0) = 0;
			mat(2, 1) = 0;
			//now set the 3rd column of the transformation to the new position with an extra 1 at the bottom
			mat(0, 2) = pos[0];
			mat(1, 2) = pos[1];
			//add an affine 1 to the bottom of the vector
			mat(2, 2) = 1;
			//so our transformation matrix should look like this:
			//[x -y  a]
			//[y  x  b]
			//[0  0  1]
			// with [a,b] corresponding to pos vector
		}
	private:

		/*
		the internal representation of a matrix
		*/
		Amat3x3 mat;
	};
	
	//operators for the other way round must be implemented as non-members

/*
scalar product
*/
	inline Vec2d operator * (real value, const Vec2d& vector)
	{
		return vector * value;
	}

	/*
	scalar sum
	*/
	inline Vec2d operator + (real value, const Vec2d& vector)
	{
		return vector + value;
	}

	/*
	scalar difference
	*/
	inline Vec2d operator - (real value, const Vec2d& vector)
	{
		return Vec2d(-vector + value);
	}


};
