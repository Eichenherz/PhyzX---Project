#pragma once

#include "Vec2.h"
#include "Matrix2.h"
#include "PX_Math.h"

struct PX_AABB 
{
	IVec2	center;
	IVec2	half_lengths;


	PX_AABB( const IVec2& pos, int width, int height )
		:
		half_lengths	{ width / 2, height / 2 },
		center			{ pos + half_lengths }
	{}
};

struct PX_OBB
{
	IVec2		center;
	IVec2		half_lengths;
	RotMtrx2	orientation;

	PX_OBB( const IVec2& pos, int width, int height, 
			const Radians& theta = Radians { 0.0f } )
		:
		half_lengths	{ width / 2, height / 2 },
		center			{ pos + half_lengths },
		orientation		{ theta.rads }
	{}

	PX_AABB  Make_AABB() const
	{
		return PX_AABB { center - half_lengths, half_lengths.x, half_lengths.y };
	}
};

bool AABB_Intersection( const PX_AABB& a, const PX_AABB& b );
bool OBB_Intersection( const PX_OBB& a, const PX_OBB& b );

class PX_Box_Shape 
{
public:
					PX_Box_Shape( const IVec2& pos, int width, int height );

	bool			Collision_Test( const PX_Box_Shape& box ) const;
	const IVec2&	Center() const;
	void			Transform( const struct PX_Pose_Data& pose );
	void			Draw( class Graphics& gfx, class Color c ) const;


	PX_OBB		OBB;
private:
	void			Rotate( float theta );
	void			Translate( const IVec2& displacement );
};
