#include "PX_Math.h"

constexpr float TWO_PI = 2 * M_PI;
constexpr float DGS_360 = 360.0f;

Degrees::Degrees( float dgs )
	:
	degrees { dgs }
{
	Normalize();
}

void Degrees::Normalize()
{
	if ( std::fabs( degrees ) >= DGS_360 )
	{
		degrees -= std::copysign( DGS_360, degrees );
	}
}

Radians::Radians( float rads )
	:
	rads { rads }
{
	Normalize();
}

Radians::Radians( const Radians & r )
	:
	rads { r.rads }
{}

Radians & Radians::operator=( const Radians & r )
{
	this->rads = r.rads;
	return *this;
}

Radians Radians::operator+( const Radians & r ) const
{
	return Radians( rads + r.rads );
}

Radians& Radians::operator+=( const Radians & r )
{
	this->rads += r.rads;
	Normalize();
	return *this;
}

void Radians::Normalize()
{
	if ( std::fabs( rads ) >= TWO_PI )
	{
		rads -= std::copysign( TWO_PI, rads );
	}
}
