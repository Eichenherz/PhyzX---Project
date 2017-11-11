#pragma once

#define _USE_MATH_DEFINES

#include <cmath>
#include "Vec2.h"

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
