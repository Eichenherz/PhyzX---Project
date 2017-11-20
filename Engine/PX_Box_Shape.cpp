#include "PX_Box_Shape.h"
#include "PX_Physical_Traits.h"
#include <math.h>
#include "Graphics.h"

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
PX_Box_Shape::PX_Box_Shape( const IVec2& pos, int side )
	:
	OBB { pos, side }
{}

bool PX_Box_Shape::Collision_Test( const PX_Box_Shape& box ) const
{
	return false;
}

const IVec2& PX_Box_Shape::Center() const
{
	return OBB.center;
}

void PX_Box_Shape::Transform( const PX_Pose_Data& pose )
{
	Translate( pose.pos );
	Rotate( pose.orientation );
}

void PX_Box_Shape::Draw( Graphics& gfx, Color c ) const
{
	IVec2 A {  OBB.half_lenght[0],   OBB.half_lenght [0] };
	IVec2 B { -OBB.half_lenght [0],  OBB.half_lenght [0] };
	IVec2 C {  OBB.half_lenght [0], -OBB.half_lenght [0] };
	IVec2 D { -OBB.half_lenght [0], -OBB.half_lenght [0] };

	//Rotate
	OBB.orientation *= A;
	OBB.orientation *= B;
	OBB.orientation *= C;
	OBB.orientation *= D;

	// Back to world coord
	A += OBB.center;
	B += OBB.center;
	C += OBB.center;
	D += OBB.center;

	gfx.Draw_Quad( A, B, C, D, c );
}

void PX_Box_Shape::Translate( const IVec2& displacement )
{
	OBB.center += displacement;
}

void PX_Box_Shape::Rotate( const Radians& theta )
{
	OBB.orientation = RotMtrx2( theta.rads );
}