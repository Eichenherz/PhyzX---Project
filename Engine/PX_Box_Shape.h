#pragma once

#include "PX_Collision.h"



class PX_Box_Shape 
{
public:
					PX_Box_Shape( const IVec2& pos, int width, int height );

	bool			Collision_Test( const PX_Box_Shape& box ) const;
	const IVec2&	Center() const;
	void			Transform( const struct PX_Pose_Data& pose );
	void			Draw( class Graphics& gfx, class Color c ) const;


	PX_OBB			OBB;
private:
	
};
