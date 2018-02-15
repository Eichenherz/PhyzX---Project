#include "PX_Collision.h"

using namespace CONSTANTS;
 static std::array<std::array<FVec2, 2>, 2 > box_face_normals {
	FVec2 { 1.0f,0.0f }, FVec2 { -1.0f, 0.0f } , // x[0,0], -x[0,1]
	FVec2 { 0.0f,1.0f }, FVec2 {  0.0f,-1.0f }   // y[1,0], -y[1,1]
};


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

	const Scalar scale( 1000 ); // Floating point error test. Potential bug for parallel face-face overlap.

	// A space
	const auto At = a.orientation.inverse() * t;
	const auto Rb = R * b.half_lengths;

	float sep_xa = std::fabs( At.x ) - ( a.half_lengths.x + Rb.x );
	if ( sep_xa > Scalar( scale ) * std::numeric_limits<Scalar>::epsilon() ) return false;

	float sep_ya = std::fabs( At.y ) - ( a.half_lengths.y + Rb.y );
	if ( sep_ya > Scalar( scale ) * std::numeric_limits<Scalar>::epsilon() ) return false;


	// B space
	const auto Bt = b.orientation.inverse() * t;
	const auto Ra = R.inverse() * a.half_lengths;//( b.orientation.inverse() * a.orientation ).make_abs() * a.half_lengths;

	float sep_xb = std::fabs( Bt.x ) - ( b.half_lengths.x + Ra.x );
	if ( sep_xb > Scalar( scale ) * std::numeric_limits<Scalar>::epsilon() ) return false;

	float sep_yb = std::fabs( Bt.y ) - ( b.half_lengths.y + Ra.y );
	if ( sep_yb > Scalar( scale ) * std::numeric_limits<Scalar>::epsilon() ) return false;

	return true;
}


/*
============================
	  GEOMERTY QUERY
============================
*/
Geometry_Query::Geometry_Query( const PX_OBB& obb )
	:
	obb { &obb }
{}

std::array<FVec2, 2> Geometry_Query::Get_Face_Vertices( const FVec2& normal ) const
{
	const FVec2 half_lengths = FVec2( obb->half_lengths );
	const FVec2 v { normal.x * half_lengths.x, normal.y * half_lengths.y };
	assert( v.x == Scalar( 0 ) || v.y == Scalar( 0 ) );

	std::array<FVec2, 2> vertices;
	if ( v.x == 0 )
	{
		vertices [0] = { -half_lengths.x, v.y };
		vertices [1] = {  half_lengths.x, v.y };
	} 
	else if( v.y == 0 )
	{
		vertices [0] = { v.x, -half_lengths.y };
		vertices [1] = { v.x,  half_lengths.y };
	}

	return vertices;
}

const RotMtrx2& Geometry_Query::Get_Coord_Frame() const //////
{
	return obb->orientation;
}

void Geometry_Query::Swap( Geometry_Query& other )
{
	assert( this->obb != other.obb );
	std::swap( this->obb, other.obb );
}


/*
============================
	   SAT NARROWPHASE
============================
*/
std::pair<Scalar, FVec2> Min_Separation_Axis( const PX_OBB& a, const PX_OBB& b )
{
	Scalar		best_distance;
	FVec2		min_sep_axis; 
	FVec2		t = a.orientation.inverse() * FVec2( b.center - a.center );				
	RotMtrx2	R = ( a.orientation.inverse() * b.orientation ).make_abs();
	
	const auto	Rb = R * b.half_lengths;
	Scalar		sep_x = std::fabs( t.x ) - ( a.half_lengths.x + Rb.x );
	Scalar		sep_y = std::fabs( t.y ) - ( a.half_lengths.y + Rb.y );

	if ( sep_x >= sep_y )// Bias_Greater_Than( sep_x, sep_y )// what if equal ? 
	{
		best_distance = sep_x;
		min_sep_axis  = std::signbit( t.x ) ? box_face_normals [0] [0] : box_face_normals [0] [1];
	} else {
		best_distance = sep_y;
		min_sep_axis  = std::signbit( t.y ) ? box_face_normals [1] [0] : box_face_normals [1] [1];
	}

	return { best_distance, min_sep_axis };
}

std::array<FVec2, 2> Find_Incident_Face( const FVec2& ref_n, const Geometry_Query& ref, const Geometry_Query& inc )
{
	// Inc reference frame
	const auto		inc_frame = inc.Get_Coord_Frame().inverse() * ref.Get_Coord_Frame();
	const FVec2		ref_normal = inc_frame * FVec2( -ref_n ); // Flip normal to obtain correct face & not the opposite one.
	FVec2			inc_face_normal;
	
	if ( std::fabs( ref_normal.x ) > std::fabs( ref_normal.y ) )
	{
		inc_face_normal = std::signbit( ref_normal.x ) ? box_face_normals [0] [1] : box_face_normals [0] [0];
	} else {
		inc_face_normal = std::signbit( ref_normal.y ) ? box_face_normals [1] [1] : box_face_normals [1] [0];
	}

	std::array<FVec2, 2> incident_face = inc.Get_Face_Vertices( inc_face_normal ); // bug
	
	// Ref reference frame
	const auto		ref_frame = inc_frame.inverse();//ref.Get_Coord_Frame().orientation.inverse() * inc.Get_Coord_Frame().orientation;
	std::for_each( incident_face.begin(), incident_face.end(),
				   [&] ( FVec2& vertex )
				   {
					   ref_frame *= vertex;
				   } );
	
	return incident_face;
}

std::array<FVec2, 2> Clip_Segment_to_Line( const Line& l, const std::array<FVec2, 2>& face ) // might need to use IVec2 only for plotting.
{
	std::array<FVec2, 2>	segment;
	int						idx = 0;
	// Distances from corresponding points to line l.
	Scalar d0 = Dot_Prod( l.normal, face [0] ) - l.c;
	Scalar d1 = Dot_Prod( l.normal, face [1] ) - l.c;
	
	// Case 1 : both points lie behind( negative ) the plane.
	if ( d0 <= Scalar( 0 ) ) { segment [idx++] = face [0]; }
	if ( d1 <= Scalar( 0 ) ) { segment [idx++] = face [1]; }

	// Case 2 : pts lie on different sides of the plane.
	if ( d0 * d1 < Scalar( 0 ) ) // less than to ignore -0.0f
	{
		// lerp to find intersection point.
		const Scalar interp = d0 / ( d0 - d1 );
		segment [idx] = face [0] * ( Scalar( 1 ) - interp ) + face [1] * interp;
		++idx;
	}
	assert( idx != 3 );
	return segment;
}

void SAT_Narrowphase( Manifold& m, const PX_OBB& a, const PX_OBB& b )
{
	auto penetrationA = Min_Separation_Axis( a, b );
	if ( penetrationA.first > 0.0f ) return;

	auto penetrationB = Min_Separation_Axis( b, a );
	if ( penetrationB.first > 0.0f ) return;

	Geometry_Query			ref { a };
	Geometry_Query			inc { b };
	FVec2					ref_face_normal;
	bool					flip = false;

	// Select ref & inc face. 
	if ( Bias_Greater_Than( penetrationA.first, penetrationB.first ) )
	{
		ref_face_normal = penetrationA.second;
	}
	else {
		ref_face_normal = penetrationB.second;
		ref.Swap( inc );
		flip = true;
	}

	// Coord frame is Ref frame.
	std::array<FVec2, 2>	inc_face_vertices = Find_Incident_Face( ref_face_normal, ref, inc );
	std::array<FVec2, 2>	ref_face_vertices = ref.Get_Face_Vertices( ref_face_normal );

	// Gather data to compute side planes  
	const FVec2				side_plane_normal = ( ref_face_vertices [1] - ref_face_vertices [0] ).Normalize();
	const Scalar			neg_plane = -Dot_Prod( side_plane_normal, ref_face_vertices [0] );
	const Scalar			pos_plane =  Dot_Prod( side_plane_normal, ref_face_vertices [1] );

	auto neg_clip = Clip_Segment_to_Line( { -side_plane_normal, neg_plane }, inc_face_vertices );
	if ( neg_clip.size() < 2 ) return;
	inc_face_vertices = std::move( neg_clip );

	auto pos_clip = Clip_Segment_to_Line( { side_plane_normal, pos_plane }, inc_face_vertices );
	if ( pos_clip.size() < 2 ) return;
	inc_face_vertices = std::move( pos_clip );

	// Generate manifold ( transform to world coords. )
	const Scalar	d_to_ref_face = Dot_Prod( ref_face_normal, ref_face_vertices [0] );
	int				ct_pts = 0;
	
	std::for_each( inc_face_vertices.begin(), inc_face_vertices.end(),
				   [&] ( const FVec2& vertex ) 
				   {
					   const Scalar separation = Dot_Prod( ref_face_normal, vertex ) - d_to_ref_face;
					   if ( separation <= Scalar( 0 ) )
					   {
						   m.contacts [ct_pts].position = IVec2( vertex );//To_World_Frame( IVec2(vertex), *inc.obb );//
						   m.contacts [ct_pts].penetration	= std::fabs( separation );
						   ++ct_pts;
					   }
				   } );
	std::for_each( m.contacts.begin(), m.contacts.end(),
				   [&] ( auto& contact ) 
				   {
					   contact.position =	ref.Get_Coord_Frame() * 
											inc.Get_Coord_Frame().inverse() * contact.position;
					  // inc.Get_Coord_Frame() *= contact.position;
					   contact.position += ref.obb->center;
				   } );
	m.normal = ( flip ) ? IVec2( -ref_face_normal ) : IVec2( ref_face_normal ); // Normal goes from ref to inc
	m.normal = To_World_Frame( m.normal, *ref.obb );

	// For debugging purposes.
	m.a = ref.obb;
	m.b = inc.obb;
}


/*
============================
	   DEBUG FUNCTIONS
============================
*/
#include "Graphics.h"
#include "Font.h"

void Manifold::Min_Sep_Axis_Debug( Graphics& gfx, const Font& f )
{
	auto queryA = Min_Separation_Axis( *( this->a ), *( this->b ) );
	auto queryB = Min_Separation_Axis( *( this->b ), *( this->a ) );

	// Should scale normal by penetration along axis.
	queryA.second *= queryA.first;
	queryB.second *= queryB.first;

	auto A_axis = To_World_Frame( queryA.second, *a );
	auto B_axis = To_World_Frame( queryB.second, *b );

	gfx.Draw_Line( a->center, IVec2( A_axis ), Colors::Cyan );
	gfx.Draw_Line( b->center, IVec2( B_axis ), Colors::Cyan );

	if ( queryA.first > queryB.first ) //Bias_Greater_Than( queryA.first, queryB.first )
	{
		f.DrawText( "BLUE", { 10, 30 }, Colors::Blue, gfx );
	}
	else
	{
		f.DrawText( "WHITE", { 10, 30 }, Colors::White, gfx );
	}
}

void Manifold::Debug_Draw( Graphics& gfx ) const
{
	// Draw collision normal from ref center
	//const auto n = normal * 10;
	//gfx.Draw_Line( a->center, normal, Colors::Yellow );

	std::for_each( contacts.cbegin(), contacts.cend(),
				   [&] ( const Contact_Point& ct_pt )
				   {
					   gfx.PutPixel( ct_pt.position.x, ct_pt.position.y, Colors::Magenta );
					} );
	// Draw contact pts as a box centered at these.
	/*
	auto contact_box_vertices = Klein_4_Vertices( 5, 5 );
	std::for_each( contact_box_vertices.begin(), contact_box_vertices.end(),
				   [&] ( IVec2& vertex ) 
				   {
					   a->orientation *= vertex;
				   } );
	std::for_each( contacts.begin(), contacts.end(),
				   [&] ( const Contact_Point& contact )
				   {
					   std::for_each( contact_box_vertices.begin(), contact_box_vertices.end(),
									  [&] ( IVec2& vertex )
									  {
										  vertex += contact.position;
									  } );
					   gfx.Draw_Quad( contact_box_vertices [0], contact_box_vertices [1], 
									  contact_box_vertices [2], contact_box_vertices [3], Colors::Green );
				   } );
	
				 //  */
}
