#pragma once

#include <cmath>
#include "Vec2.h"
#include "Matrix2.h"
#include <array>

using Scalar = float;

namespace CONSTANTS {
	constexpr Scalar E			( 2.71828182845904523536 );
	constexpr Scalar LOG2E		( 1.44269504088896340736 );
	constexpr Scalar LOG10E		( 0.434294481903251827651 );
	constexpr Scalar LN2		( 0.693147180559945309417 );
	constexpr Scalar LN10		( 2.30258509299404568402 );
	constexpr Scalar PI			( 3.14159265358979323846 );
	constexpr Scalar TWO_PI		( 6.283185307179586 );
	constexpr Scalar PI_OVER_2	( 1.57079632679489661923 );
	constexpr Scalar PI_OVER_4	( 0.785398163397448309616 );
	constexpr Scalar SQRT2		( 1.41421356237309504880 );
	constexpr Scalar SQRT1_2	( 0.707106781186547524401 );
	constexpr Scalar DGS_360	( 360.0 );
};


struct Radians
{
	Scalar rads;

				Radians() = default;
				Radians( Scalar rads );
				Radians( const Radians& r );
				~Radians() = default;
	Radians&	operator=( const Radians& r );
	Radians		operator+( const Radians& r ) const;
	Radians&	operator+=( const Radians& r );
	
	//To be added as needed
	void		Normalize();
};

std::array<IVec2, 4> Klein_4_Vertices( int x, int y );
std::array<IVec2, 4> Klein_4_Faces( const std::array<IVec2, 4>& vertices );
std::array<IVec2, 4> Klein_4_Normals( const std::array<IVec2, 4>& vertices );

bool Bias_Greater_Than( Scalar a, Scalar b );