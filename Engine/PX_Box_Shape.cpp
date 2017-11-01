#include "PX_Box_Shape.h"
#include <math.h>
#include "Graphics.h"

/*  COLLISION OF 2 AABBs  */
bool AABB_Intersection( const PX_AABB& a, const PX_AABB& b )
{
	return	std::abs( b.center.x - a.center.x ) < ( b.radius + a.radius ) &&
		std::abs( b.center.y - a.center.y ) < ( b.radius + a.radius );
}

//======================================================================//
//																		//
//						METHODS OF CLASS:								//
//						  PX_Box_Shape									//
//																		//
//======================================================================//
PX_Box_Shape::PX_Box_Shape( IVec2 pos, int side )
	:
	AABB { pos, side }
{}

//==============================================================//
//																//
//		COLLISION OF EMBEDDED AABB & OTHER EMBEDDED AABB:		//
//																//
//==============================================================//
bool PX_Box_Shape::Collision_Test( const PX_Box_Shape& box ) const
{
	return	AABB_Intersection( this->AABB, box.AABB );
}

IVec2 PX_Box_Shape::Center() const
{
	return AABB.center;
}

void PX_Box_Shape::Transformation( /* params */ const IVec2& displacement )
{
	Rotate();
	Translate( displacement );
}

const int PX_Box_Shape::Area() const
{
	return ( 2 * AABB.radius ) * ( 2 * AABB.radius );
}

void PX_Box_Shape::Draw( Graphics& gfx, Color c ) const
{
	const int x_start = AABB.center.x - AABB.radius;
	const int x_end	  = AABB.center.x + AABB.radius;
	const int y_start = AABB.center.y - AABB.radius;
	const int y_end	  = AABB.center.y + AABB.radius;

	for ( auto x = x_start; x < x_end; ++x )
		for ( auto y = y_start; y < y_end; ++y )
			gfx.PutPixel( x, y, c );
}

void PX_Box_Shape::Rotate()
{
}

void PX_Box_Shape::Translate( const IVec2& displacement )
{
	AABB.center += displacement;
}

