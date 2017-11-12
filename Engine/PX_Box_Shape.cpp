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
	OBB { pos, side }
{}

//==============================================================//
//																//
//		COLLISION OF EMBEDDED AABB & OTHER EMBEDDED AABB:		//
//																//
//==============================================================//
bool PX_Box_Shape::Collision_Test( const PX_Box_Shape& box ) const
{
	//return	AABB_Intersection( this->AABB, box.AABB );
	return false;
}

IVec2 PX_Box_Shape::Center() const
{
	return OBB.center;
}

void PX_Box_Shape::Transformation( const IVec2& displacement, float theta )
{
	Rotate( theta );
	Translate( displacement );
}

void PX_Box_Shape::Draw( Graphics& gfx, Color c ) const
{
	const int x_start = OBB.center.x - OBB.radius;
	const int y_start = OBB.center.y - OBB.radius;
	const int x_end	  = OBB.center.x + OBB.radius;
	const int y_end	  = OBB.center.y + OBB.radius;

	IVec2 A { x_start, y_start };
	IVec2 B { x_start, y_end };
	IVec2 C { x_end, y_start };
	IVec2 D { x_end, y_end };

	//IVec2 A = OBB.center - IVec2 { OBB.radius, OBB.radius };
	//IVec2 B = A + IVec2 { 0, OBB.radius };

	// Obj coord space
	A -= OBB.center;
	B -= OBB.center;
	C -= OBB.center;
	D -= OBB.center;
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

void PX_Box_Shape::Rotate( float theta )
{
}

void PX_Box_Shape::Translate( const IVec2& displacement )
{

}

