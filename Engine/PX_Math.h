#pragma once

#include <math.h>
#include "Vec2.h"

struct Angle_Degrees
{
	float degrees;

	//Work in progress. To be added as needed.

			Angle_Degrees( float dgs );

	void	Normalize_360();
};

