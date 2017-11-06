#include "PX_Math.h"

Angle_Degrees::Angle_Degrees( float dgs )
	:
	degrees { dgs }
{
	Normalize_360();
}

void Angle_Degrees::Normalize_360()
{
	if ( std::fabs( degrees ) >= 360 )
	{
		if ( std::signbit( degrees ) ) degrees += 360;
		else degrees -= 360;
	}
}
