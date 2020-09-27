#pragma once
#include <armadillo>
#include <float.h>
#define _USE_MATH_DEFINES

namespace phy {
	/*
	decides the precision of the whole engine
	*/
	typedef float real;

	/*
	decides the internal representation of vectors
	*/
	typedef arma::fvec2 Avec2d;

	/*
	decides the internal representation of matrices
	*/
	typedef arma::fmat33 Amat3x3;

	/*
	decides the internal representation of matrices
	*/
	typedef arma::fmat22 Amat2x2;

	/*
	various precision functions and values
	*/
	#define real_abs fabsf;
	#define REAL_MAX FLT_MAX;
	#define real_cos cos; // wont work
	#define real_sin sin; // wont work dont use
	#define real_pow powf;
	const int MAX_VERTS = 10;
	constexpr double PI = 3.1415926;
}