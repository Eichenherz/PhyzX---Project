#pragma once

#include "Vec2.h"
#include "Matrix2.h"

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
};

bool AABB_Intersection( const PX_AABB& a, const PX_AABB& b );

class PX_Box_Shape 
{
public:
				PX_Box_Shape( IVec2 pos, int side );

	bool		Collision_Test( const PX_Box_Shape& box ) const;
	IVec2		Center() const;
	void		Transformation( /* params */ const IVec2& displacement );
	const int	Area() const;

	/* METHODS FOR TESTING */
	void				Draw( class Graphics& gfx, class Color c ) const;
	const PX_AABB&		Get_AABB() const
	{
		return AABB;
	}
	void				Move_Box( const FVec2& new_pos)
	{
		if ( new_pos.x != 0.0f )
			AABB.center.x += int( new_pos.x );
		if ( new_pos.y != 0.0f )
			AABB.center.y += int( new_pos.y );
	}

private:
	PX_AABB AABB;

	void Rotate();
	void Translate( const IVec2& displacement );
};
