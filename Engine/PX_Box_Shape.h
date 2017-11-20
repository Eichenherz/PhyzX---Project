#pragma once

#include "Vec2.h"
#include "Matrix2.h"
#include "PX_Math.h"
#include <array>

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
	IVec2				center;
	RotMtrx2			orientation;
	std::array<int, 2>	half_lenght;

	PX_OBB( const IVec2& center, int width,
			int height = 0, const Radians& theta = Radians { 0.0f } )
		:
		center { center },
		orientation { theta.rads }
	{
		half_lenght [0] = width / 2;
		half_lenght [1] = height / 2;
	}
};

bool AABB_Intersection( const PX_AABB& a, const PX_AABB& b );

class PX_Box_Shape 
{
public:
					PX_Box_Shape( const IVec2& pos, int side );

	bool			Collision_Test( const PX_Box_Shape& box ) const;
	const IVec2&	Center() const;
	void			Transform( const struct PX_Pose_Data& pose );
	void			Draw( class Graphics& gfx, class Color c ) const;

private:
	PX_OBB OBB;

	void			Rotate( const Radians& theta );
	void			Translate( const IVec2& displacement );
};
