#pragma once

#include <math.h>
#include "Vec2.h"

static constexpr float h = 0.005f; // EULER INTEGRATOR constant

template<typename T>
Vec2_<T>& Euler_Integrator( float dt, const Vec2_<T>& itr0 = { (T) 0,(T) 0 } )
{
	Vec2_<T> itr_next = itr0 + h * itr0 / dt;

	return iter_next;
}

