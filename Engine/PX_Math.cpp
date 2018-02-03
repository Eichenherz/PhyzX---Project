#include "PX_Math.h"

using namespace CONSTANTS;

Radians::Radians( Scalar rads )
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

Scalar Point_Point_Distance_Sq( const IVec2 & p1, const IVec2 & p2 )
{
	auto v = ( p1 - p2 );
	return Dot_Prod( v, v );
}

std::array<IVec2, 4> Klein_4_Vertices( int x, int y )
{
	std::array<IVec2, 4> k4;
	for ( size_t i = 0; i < k4.size(); ++i )
	{
		const auto sgn_x = cos( PI_OVER_4 + i * PI_OVER_2 );
		const auto sgn_y = sin( PI_OVER_4 + i * PI_OVER_2 );

		k4 [i] = IVec2 { int(std::copysign( x, sgn_x )),
						 int(std::copysign( y, sgn_y ))};
	} 
	return k4;
}

std::array<IVec2, 4> Klein_4_Faces( const std::array<IVec2, 4>& vertices )
{
	std::array<IVec2, 4> faces;
	for ( size_t i = 0, j = i + 1; i < vertices.size(); ++i )
	{
		j = ( j + 1 == vertices.size() ) ? 0 : ++j;
		IVec2 face = ( vertices [j] - vertices [i] ).Normalize();

		faces [i] = face;
	}
	return faces;
}

std::array<IVec2, 4> Klein_4_Normals( const std::array<IVec2, 4>& vertices )
{
	std::array<IVec2, 4> normals;
	for ( size_t i = 0, j = i + 1; i < vertices.size(); ++i ) //for_each....lambda
	{
		j = ( j + 1 == vertices.size() ) ? 0 : ++j;
		IVec2 temp = (vertices [j] - vertices [i]).Normalize();
		Matrix2( 0.0f, -1.0f, 1.0f, 0.0f ) *= temp;

		normals [i] = temp;
	}
	return normals;
}

bool Bias_Greater_Than( Scalar a, Scalar b )
{
	constexpr Scalar relative_bias = 0.95f;
	constexpr Scalar absolute_bias = 0.01f; 

	// >= instead of > for NaN comparison safety
	return a >= b * relative_bias + a * absolute_bias;
}


