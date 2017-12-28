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
	FVec2		t = FVec2( b.center - a.center );
	RotMtrx2	R = a.orientation.inverse() * b.orientation;

	R.make_abs();

	// A space
	const auto At = a.orientation.inverse() * t;
	const auto Rb = R * b.half_lengths;

	float sep_xa = std::fabs( At.x ) - ( a.half_lengths.x + Rb.x );
	if ( sep_xa > 0.0f ) return false;

	float sep_ya = std::fabs( At.y ) - ( a.half_lengths.y + Rb.y );
	if ( sep_ya > 0.0f ) return false;
	
	// B space
	const auto Bt = b.orientation.inverse() * t;
	const auto Ra = R.inverse() * a.half_lengths;

	float sep_xb = std::fabs( Bt.x ) - ( b.half_lengths.x + Ra.x );
	if ( sep_xb > 0.0f ) return false;

	float sep_yb = std::fabs( Bt.y ) - ( b.half_lengths.x + Ra.y );
	if ( sep_yb > 0.0f ) return false;

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