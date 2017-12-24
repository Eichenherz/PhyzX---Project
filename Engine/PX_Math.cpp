#include "PX_Math.h"
#include <complex>

constexpr float TWO_PI = 2.0f * M_PI;
constexpr float PI = M_PI;
constexpr float PI_OVER_2 = M_PI / 2.0f;
constexpr float PI_OVER_4 = M_PI / 4.0f;
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

IVec2 Sgn_Alternator( int i )
{
	return
	{ int( cos( i * PI_OVER_2 ) ),
	  int( sin( PI_OVER_2 + i * PI_OVER_2 ) ) };
}

std::array<IVec2, 4> Klein_4( int x, int y )
{
	std::array<IVec2, 4> k4;
	for ( int i = 0; i < 4; ++i )
	{
		const auto sgn_x = sin( PI_OVER_2 + i * PI );
		const auto sgn_y = sin( PI_OVER_4 + i * PI_OVER_2 );

		k4 [i] = IVec2 { int(std::copysign( x, sgn_x )),
						 int(std::copysign( y, sgn_y ))};
	} 
	return k4;
}
