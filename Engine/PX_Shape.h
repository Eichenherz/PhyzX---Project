#pragma once

#include "Vec2.h"

class PX_Shape
{
public:
	virtual			~PX_Shape(){};

	virtual bool	Contains( const FVec2& point ) const = 0;
	virtual bool	Overlapping_With( const PX_Shape& shape ) const = 0;
	virtual FVec2	Intersection_Point_Vec( const FVec2& ) const = 0;
	virtual FVec2	Center() = 0;
};

