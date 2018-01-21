#include "PX_Collision.h"

using namespace CONSTANTS;

/*
============================
			AABB
============================
*/
PX_AABB::PX_AABB( const IVec2& pos, int width, int height )
	:
	half_lengths	{ width / 2, height / 2 },
	center			{ pos + half_lengths }
{}

bool AABB_Intersection( const PX_AABB& a, const PX_AABB& b )
{
	return
		std::abs( b.center.x - a.center.x ) < ( b.half_lengths.x + a.half_lengths.x ) &&
		std::abs( b.center.y - a.center.y ) < ( b.half_lengths.y + a.half_lengths.y );
}


/*
============================
			OBB
============================
*/
PX_OBB::PX_OBB( const IVec2& pos, int width, int height, const Radians& theta )
	:
	half_lengths	{ width / 2, height / 2 },
	center			{ pos + half_lengths },
	orientation		{ theta }
{}

bool OBB_Intersection( const PX_OBB& a, const PX_OBB& b )
{
	FVec2		t = FVec2( b.center - a.center );
	RotMtrx2	R = ( a.orientation.inverse() * b.orientation ).make_abs();


	// A space
	const auto At = a.orientation.inverse() * t;
	const auto Rb = R * b.half_lengths;

	float sep_xa = std::fabs( At.x ) - ( a.half_lengths.x + Rb.x );
	if ( sep_xa > 0.0f ) return false;

	float sep_ya = std::fabs( At.y ) - ( a.half_lengths.y + Rb.y );
	if ( sep_ya > 0.0f ) return false;


	// B space
	const auto Bt = b.orientation.inverse() * t;
	const auto Ra = R.inverse() * a.half_lengths;//( b.orientation.inverse() * a.orientation ).make_abs() * a.half_lengths;

	float sep_xb = std::fabs( Bt.x ) - ( b.half_lengths.x + Ra.x );
	if ( sep_xb > 0.0f ) return false;

	float sep_yb = std::fabs( Bt.y ) - ( b.half_lengths.x + Ra.y );
	if ( sep_yb > 0.0f ) return false;

	return true;
}


/*
============================
	   SAT NARROWPHASE
============================
*/
Geometry_Query::Geometry_Query( const PX_OBB& obb )
	:
	obb { &obb }
{
}

IVec2 Geometry_Query::Get_Vetrex( Traits_ID idx ) const
{
	const auto sgn_x = cos( PI_OVER_4 + int( idx ) * PI_OVER_2 );
	const auto sgn_y = sin( PI_OVER_4 + int( idx ) * PI_OVER_2 );

	return IVec2 { int( std::copysign( obb->half_lengths.x, sgn_x ) ),
				   int( std::copysign( obb->half_lengths.y, sgn_y ) ) };
}

std::array<IVec2, 2> Geometry_Query::Get_Face_Vertices( Traits_ID idx ) const
{
	std::array<IVec2, 2> vertices;

	vertices [0] = Get_Vetrex( idx );
	idx = ( idx == Traits_ID::T4 ) ? Traits_ID::T1 : Traits_ID( int( idx ) + 1 );
	vertices [1] = Get_Vetrex( idx );

	return vertices;
}

IVec2 Geometry_Query::Get_Face_Normal( Traits_ID idx ) const
{
	const int x = (int) cos( int( idx ) * PI_OVER_2 );
	const int y = (int) sin( int( idx ) * PI_OVER_2 );
	return IVec2 { x, y };
}

const PX_OBB& Geometry_Query::Get_OBB() const //////
{
	auto temp = &obb;
	return reinterpret_cast<const PX_OBB&>(temp);
}

void Geometry_Query::Swap( Geometry_Query& other )
{
	assert( this->obb == other.obb );
	std::swap( this->obb, other.obb );
}

std::pair<Scalar, Traits_ID> Min_Separation_Axis( const PX_OBB& a, const PX_OBB& b )
{
	Scalar			best_distance;
	Traits_ID		min_sep_axis;
	FVec2			t = a.orientation.inverse() * FVec2( b.center - a.center );
	RotMtrx2		R = ( a.orientation.inverse() * b.orientation ).make_abs();
	

	const auto	Rb = R * b.half_lengths;
	Scalar		sep_x = std::fabs( t.x ) - ( a.half_lengths.x + Rb.x );
	Scalar		sep_y = std::fabs( t.y ) - ( a.half_lengths.y + Rb.y );

	
	if ( Bias_Greater_Than( sep_x, sep_y ) )//sep_x > sep_y // what if equal ? 
	{
		best_distance = sep_x;
		min_sep_axis  = std::signbit( t.x ) ? Traits_ID::T2 : Traits_ID::T4;
	}
	else 
	{
		best_distance = sep_y;
		min_sep_axis  = std::signbit( t.y ) ? Traits_ID::T3 : Traits_ID::T1;
	}

	return { best_distance, min_sep_axis };
}

std::array<IVec2, 2> Find_Incident_Face( const IVec2& ref_n, const Geometry_Query& ref, const Geometry_Query& inc )
{
	// Inc reference frame
	const auto		inc_frame = inc.Get_OBB().orientation.inverse() * ref.Get_OBB().orientation;
	FVec2			ref_normal = inc_frame * FVec2( -ref_n ); // Flip normal to obtain correct face & not the opposite one.
	Traits_ID		inc_face_idx;

	if ( Bias_Greater_Than( std::fabs( ref_normal.x ), std::fabs( ref_normal.y ) ))
	{
		inc_face_idx = ( std::signbit( ref_normal.x ) ) ? Traits_ID::T3 : Traits_ID::T1;
	}
	else
	{
		inc_face_idx = ( std::signbit( ref_normal.y ) ) ? Traits_ID::T4 : Traits_ID::T2;
	}

	std::array<IVec2, 2> incident_face = inc.Get_Face_Vertices( inc_face_idx );
	
	// Ref reference frame
	const auto		ref_frame = inc_frame.inverse();//ref.Get_OBB().orientation.inverse() * inc.Get_OBB().orientation;
	std::for_each( incident_face.begin(), incident_face.end(),
				   [&] ( IVec2& vertex )
				   {
					   ref_frame *= vertex;
				   } );
	
	return incident_face;
}

void Clip( const IVec2 & ref_n, const IVec2* const ref_face, const IVec2* inc_face )
{
	// Compute side planes
	Scalar pos_plane =  Dot_Prod( ref_face [1] - ref_face [0], ref_face [1] );
	Scalar neg_plane = -Dot_Prod( ref_face [1] - ref_face [0], ref_face [0] );


}

void SAT( const PX_OBB& a, const PX_OBB& b )
{
	auto penetrationA = Min_Separation_Axis( a, b );
	if ( penetrationA.first > 0.0f ) return;

	auto penetrationB = Min_Separation_Axis( b, a );
	if ( penetrationB.first > 0.0f ) return;


	Geometry_Query			ref { a };
	Geometry_Query			inc { b };
	Scalar					penetration_depth;
	Traits_ID				trait_id;
	
	if ( Bias_Greater_Than( penetrationA.first, penetrationB.first ) )
	{
		penetration_depth = penetrationA.first;
		trait_id = penetrationA.second;
	}
	else // need to flip sign when passing to manifold
	{
		penetration_depth = penetrationB.first;
		trait_id = penetrationB.second;
		ref.Swap( inc ); 
	}
 
	IVec2					ref_face_normal = ref.Get_Face_Normal( trait_id );
	std::array<IVec2, 2>	ref_face_vertices = ref.Get_Face_Vertices( trait_id );
	std::array<IVec2, 2>	inc_face_vertices = Find_Incident_Face( ref_face_normal, ref, inc );
}
