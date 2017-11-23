#include "PX_Box_Shape.h"
#include "PX_Physical_Traits.h"
#include <math.h>
#include "Graphics.h"
#include "array"

bool AABB_Intersection( const PX_AABB& a, const PX_AABB& b )
{
	return	
		std::abs( b.center.x - a.center.x ) < ( b.half_lengths.x + a.half_lengths.x ) &&
		std::abs( b.center.y - a.center.y ) < ( b.half_lengths.y + a.half_lengths.y );
}

bool OBB_Intersection( const PX_OBB& a, const PX_OBB& b )
{
	auto temp = b.half_lengths;
	b.orientation.inverted() *= temp;
	a.orientation *= temp;

	auto flag_a = AABB_Intersection( a.Make_AABB(), PX_AABB { b.center, temp.x,temp.y } );

	temp = a.half_lengths;
	a.orientation.inverted() *= temp;
	b.orientation *= temp;

	auto flag_b = AABB_Intersection( b.Make_AABB(), PX_AABB { a.center, temp.x,temp.y } );

	return flag_a && flag_b;
}


//======================================================================//
//																		//
//						METHODS OF CLASS:								//
//						  PX_Box_Shape									//
//																		//
//======================================================================//
PX_Box_Shape::PX_Box_Shape( const IVec2& pos, int width, int height )
	:
	OBB { pos, width, height }
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
	IVec2 A =  OBB.half_lengths;
	IVec2 B { -OBB.half_lengths.x,  OBB.half_lengths.y };
	IVec2 C {  OBB.half_lengths.x, -OBB.half_lengths.y };
	IVec2 D = -OBB.half_lengths;

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
	OBB.orientation = RotMtrx2( theta.rads );//
	// *= for nice effect;
}