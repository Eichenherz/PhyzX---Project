#include "PX_Box_Shape.h"
#include "PX_Physical_Traits.h"
#include <math.h>
#include "Graphics.h"



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
	auto vertices = Klein_4_Vertices( OBB.half_lengths.x, OBB.half_lengths.y );
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