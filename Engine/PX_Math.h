#pragma once

#define _USE_MATH_DEFINES

#include <cmath>
#include "Vec2.h"
#include <array>

struct Degrees
{
	float degrees;

	//Work in progress. To be added as needed.

			Degrees( float dgs );

	void	Normalize();
};

struct Radians
{
	float rads;

				Radians() = default;
				Radians( float rads );
				Radians( const Radians& r );
				~Radians() = default;
	Radians&	operator=( const Radians& r );
	Radians		operator+( const Radians& r ) const;
	Radians&	operator+=( const Radians& r );
	
	//To be added as needed
	void		Normalize();
};

IVec2 Sgn_Alternator( int i );

std::array<IVec2, 4> Klein_4( int x, int y );