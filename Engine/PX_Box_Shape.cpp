#include "PX_Box_Shape.h"
#include <math.h>
#include "Graphics.h"


PX_Box_Shape::PX_Box_Shape( IVec2 pos, int side )
	:
	AABB { pos, side }
{}

bool PX_Box_Shape::Collision_Test( const PX_AABB& box ) const
{
	return	std::abs( box.center.x - this->AABB.center.x ) < ( this->AABB.radius + box.radius ) &&
			std::abs( box.center.y - this->AABB.center.y ) < ( this->AABB.radius + box.radius );
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

