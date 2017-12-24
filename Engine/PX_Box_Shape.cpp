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
	IVec2		t = a.orientation.inverse() * ( b.center - a.center );
	RotMtrx2	C =  b.orientation * a.orientation.inverse();
	IVec2		B = C.abs() * b.half_lengths;

	float s_aX = std::fabs( Dot_Prod( t, a.orientation.Basis_X() ) ) - ( a.half_lengths.x + ( Dot_Prod( B, a.orientation.Basis_X() ) ) );
	if ( s_aX > 0.05f ) return false;

	float s_aY = std::fabs( Dot_Prod( t, a.orientation.Basis_Y() ) ) - ( a.half_lengths.y + ( Dot_Prod( B, a.orientation.Basis_Y() ) ) );
	if ( s_aY > 0.05f ) return false;

	RotMtrx2	C1 = a.orientation * b.orientation.inverse();
	IVec2		A = C1.abs() * a.half_lengths;

	float s_bX = std::fabs( Dot_Prod( t, b.orientation.Basis_X() ) ) - ( b.half_lengths.x + ( Dot_Prod( A, b.orientation.Basis_X() ) ) );
	if ( s_bX > 0.05f ) return false;

	float s_bY = std::fabs( Dot_Prod( t, b.orientation.Basis_Y() ) ) - ( b.half_lengths.y + ( Dot_Prod( A, b.orientation.Basis_Y() ) ) );
	if ( s_bY > 0.05f ) return false;
	
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