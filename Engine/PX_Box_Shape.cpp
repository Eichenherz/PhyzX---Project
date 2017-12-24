#include "PX_Box_Shape.h"
#include "PX_Physical_Traits.h"
#include <math.h>
#include "Graphics.h"
#include "Rect.h"

bool AABB_Intersection( const PX_AABB& a, const PX_AABB& b )
{
	return	
		std::abs( b.center.x - a.center.x ) < ( b.half_lengths.x + a.half_lengths.x ) &&
		std::abs( b.center.y - a.center.y ) < ( b.half_lengths.y + a.half_lengths.y );
}

bool OBB_Intersection( const PX_OBB& a, const PX_OBB& b )
{
	float ra, rb;
	FVec2		t = FVec2( b.center - a.center );
	RotMtrx2	R;

	R.a [0] [0] = Dot_Prod( a.orientation.Basis_X(), b.orientation.Basis_X() );
	R.a [0] [1] = Dot_Prod( a.orientation.Basis_X(), b.orientation.Basis_Y() );
	R.a [1] [0] = Dot_Prod( a.orientation.Basis_Y(), b.orientation.Basis_X() );
	R.a [1] [1] = Dot_Prod( a.orientation.Basis_Y(), b.orientation.Basis_Y() );

	R.make_abs();

	t = FVec2 { Dot_Prod( t, a.orientation.Basis_X() ),
				Dot_Prod( t, a.orientation.Basis_Y() ) };

	ra = a.half_lengths.x;
	rb = b.half_lengths.x * R.a [0] [0] + b.half_lengths.y * R.a [0] [1];
	if ( std::abs( t.x ) > ra + rb ) return false;

	ra = a.half_lengths.y;
	rb = b.half_lengths.x * R.a [1] [0] + b.half_lengths.y * R.a [1] [1];
	if ( std::abs( t.y ) > ra + rb ) return false;
	


	ra = a.half_lengths.x * R.a [0] [0] + a.half_lengths.y * R.a [1] [0];
	rb = b.half_lengths.x;
	if ( std::abs( t.x * R.a [0] [0] + t.y * R.a [1] [0] ) > ra + rb ) return false;

	ra = a.half_lengths.x * R.a [0] [1] + a.half_lengths.y * R.a [1] [1];
	rb = b.half_lengths.y;
	if ( std::abs( t.x * R.a [0] [1] + t.y * R.a [1] [1] ) > ra + rb ) return false;

	return true;
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
	Rotate( pose.orientation.rads );
}

void PX_Box_Shape::Draw( Graphics& gfx, Color c ) const
{
	// A, B, C, D
	auto vertices = Klein_4( OBB.half_lengths.x, OBB.half_lengths.y );
	std::for_each( vertices.begin(), vertices.end(), 
				   [&] ( IVec2& vertex ) 
				   {
					   OBB.orientation *= vertex;
					   vertex += OBB.center;
				   } );

	gfx.Draw_Quad( vertices [0], vertices [1], vertices [2], vertices [3], c );

	//Debug
	//IVec2 start_point = OBB.center - OBB.half_lengths * 4;
	//gfx.Draw_Line( start_point, IVec2( OBB.orientation.Basis_X() * 200 ) + start_point, Colors::Yellow );
	//gfx.Draw_Line( start_point, IVec2( OBB.orientation.Basis_Y() * 200 ) + start_point, Colors::Yellow );

	gfx.Draw_Line( OBB.center, vertices[0], Colors::Magenta );
}

void PX_Box_Shape::Translate( const IVec2& displacement )
{
	OBB.center += displacement;
}

void PX_Box_Shape::Rotate( float theta )
{
	OBB.orientation = RotMtrx2( theta );//
	// *= for nice effect;
}