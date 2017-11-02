#include "PX_Math.h"

Angle_Degrees::Angle_Degrees( float dgs )
	:
	degrees { dgs }
{
	Normalize_360();
}

void Angle_Degrees::Normalize_360()
{
	if ( std::abs(degrees) > 360 )
		degrees -= 360;
}
