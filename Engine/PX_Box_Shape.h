#pragma once

#include "Vec2.h"
#include "Matrix2.h"
#include "PX_Math.h"

struct PX_AABB
{
	IVec2	center;
	int		radius; // half the widh of the square box

	PX_AABB( IVec2 pos, int side )
		:
		radius { side / 2 },
		center { pos.x + radius, pos.y + radius }
	{}
};

struct PX_OBB
{
	IVec2		center;
	RotMtrx2	orientation;
	int			radius;

	PX_OBB( const IVec2& pos, int side, float theta = 0.0f )
		:
		radius			{ side / 2 },
		center			{ pos.x + radius, pos.y + radius },
		orientation		{ theta }
	{}
};

bool AABB_Intersection( const PX_AABB& a, const PX_AABB& b );

class PX_Box_Shape 
{
public:
				PX_Box_Shape( IVec2 pos, int side );

	bool		Collision_Test( const PX_Box_Shape& box ) const;
	IVec2		Center() const;
	void		Transformation( const IVec2& displacement, float theta );
	const int	Area() const;

	/* METHODS FOR TESTING */
	void		Draw( class Graphics& gfx, class Color c ) const;
	void		Move_Box( const FVec2& new_pos)
	{
		if ( new_pos.x != 0.0f )
			OBB.center.x += int( new_pos.x );
		if ( new_pos.y != 0.0f )
			OBB.center.y += int( new_pos.y );
	}
	void		Apply_Rotation( Radians r )
	{
		OBB.orientation = RotMtrx2( r.rads );
	}
private:
	PX_OBB OBB;

	void Rotate( float theta );
	void Translate( const IVec2& displacement );
};
