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
struct PX_OBB
{
	IVec2		center;
	IVec2		half_lengths;
	RotMtrx2	orientation;


							PX_OBB( const IVec2& pos, int width, int height,
									const Radians& theta = Radians { 0.0f } );
};

bool OBB_Intersection( const PX_OBB& a, const PX_OBB& b );

/*
============================
	   SAT NARROWPHASE
============================
*/
enum class Traits_ID
{
	//
	//	VERTICES	----	 FACES		----	 FACE NORMALS
	//     y		----	   y     	----	      2
	//   3---2 		----	 + 2 +		----		+ ^ +
	//   |   |  x	----	 3   1  x   ----	  3 <   > 1
	//   4---1 		----	 + 4 +		----		+ v +
	//				----				----		  4

	/*X_pos*/ T1,
	/*Y_pos*/ T2,
	/*X_neg*/ T3,
	/*Y_neg*/ T4
};
class Geometry_Query
{
public:
							Geometry_Query( const PX_OBB& obb ); //beware !
	IVec2					Get_Vetrex( Traits_ID idx ) const;
	std::array<IVec2, 2>	Get_Face_Vertices( Traits_ID idx ) const;
	IVec2					Get_Face_Normal( Traits_ID idx ) const;
	const PX_OBB&			Get_OBB() const;// change to const PX_OBB* if & doesn't work
	void					Swap( Geometry_Query& other );

private:
	const PX_OBB* obb;
};

std::pair<Scalar, Traits_ID> Min_Separation_Axis( const PX_OBB& a, const PX_OBB& b );

std::array<IVec2, 2> Find_Incident_Face( const IVec2& ref_n, const PX_OBB& a, const PX_OBB& b );

void Clip( const IVec2& ref_n, const IVec2* const ref_face, const IVec2* inc_face );

void SAT( const PX_OBB& a, const PX_OBB& b );