#pragma once

#include "Vec2.h"
#include "Graphics.h"

class PX_Shape
{
public:
	virtual			~PX_Shape(){};

	virtual bool	Contains( const IVec2& point ) const = 0;
	virtual bool	Overlapping_With( const PX_Shape& shape ) const = 0;
	virtual IVec2	Intersection_Point_Vec( const IVec2& ) const = 0;
	virtual IVec2	Center() const = 0;
	virtual void	Draw( Graphics& gfx ) const = 0;
	virtual void	Update( float  ) = 0;
};

