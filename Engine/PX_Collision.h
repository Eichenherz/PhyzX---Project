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
	  GEOMERTY QUERY
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
	const RotMtrx2&			Get_Coord_Frame() const;
	void					Swap( Geometry_Query& other );
	

//private: //public for debugging
	const PX_OBB* obb;
};


/*
============================
	 COLLISION MANIFOLD
============================
*/
struct Contact_Point
{
	IVec2	position;
	Scalar	penetration;
};

struct Manifold
{
	const PX_OBB*					a; //ref
	const PX_OBB*					b; //inc

	std::array<Contact_Point, 2>	contacts;
	IVec2							normal;
	//IVec2							ref_center;// for debugging purposes.

	void							Debug_Draw( class Graphics& gfx ) const;
};

/*
============================
	   SAT NARROWPHASE
============================
*/
std::pair<Scalar, Traits_ID> Min_Separation_Axis( const PX_OBB& a, const PX_OBB& b );

std::array<IVec2, 2> Find_Incident_Face( const IVec2& ref_n, const Geometry_Query& a, const Geometry_Query& b );

std::array<IVec2, 2> Clip_Segment_to_Line( const Line& l, const std::array<IVec2, 2>& face );

void SAT( Manifold& m, const PX_OBB& a, const PX_OBB& b );