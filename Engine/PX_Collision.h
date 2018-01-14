#pragma once

#include "Vec2.h"
#include "Matrix2.h"
#include "PX_Math.h"
#include <utility>

/*
============================
		    AABB
============================
*/
struct PX_AABB
{
	IVec2	center;
	IVec2	half_lengths;


	PX_AABB( const IVec2& pos, int width, int height );
};

bool AABB_Intersection( const PX_AABB& a, const PX_AABB& b );

/*
============================
			OBB
============================
*/
enum class Tratis_Index
{
	//
	//	VERTICES	----	 FACES		----	 FACE NORMALS
	//     y		----	   y     	----	      1
	//   2---1 		----	 + 1 +		----		+ ^ +
	//   |   |  x	----	 2   4  x   ----	  2 <   > 4
	//   3---4 		----	 + 3 +		----		+ v +
	//				----				----		  3

	T1 = 1,
	T2 = 2,
	T3 = 3,
	T4 = 4
};

struct PX_OBB
{
	IVec2		center;
	IVec2		half_lengths;
	RotMtrx2	orientation;


							PX_OBB( const IVec2& pos, int width, int height,
									const Radians& theta = Radians { 0.0f } );
	// Shape geometry data
	IVec2					Get_Vetrex( Tratis_Index idx ) const;
	std::array<IVec2, 2>	Get_Face_Vertices( Tratis_Index idx ) const;
	IVec2					Get_Face_Normal( Tratis_Index idx ) const;
};


bool OBB_Intersection( const PX_OBB& a, const PX_OBB& b );

std::pair<Scalar, Tratis_Index> Min_Separation_Axis( const PX_OBB& a, const PX_OBB& b );

std::array<IVec2, 2> Find_Incident_Face( const IVec2& ref_n, const PX_OBB& a, const PX_OBB& b );

void Clip( const IVec2& ref_n, const IVec2* const ref_face, const IVec2* inc_face );

void SAT( const PX_OBB& a, const PX_OBB& b );