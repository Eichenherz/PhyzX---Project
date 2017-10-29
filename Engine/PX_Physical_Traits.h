#pragma once

#include "Vec2.h"
#include "cmath"

struct PX_Impulse_Data
{
	IVec2	pos;

	float	mass;
	FVec2	vel;
	float	speed;
};

class PX_Collision_Box
{
public:
	PX_Collision_Box( float speed = 0.0f, float mass = INFINITY );

	
	
private:
	PX_Impulse_Data			box_impulse;
	static constexpr float	damping_factor = 0.15f;
	static constexpr float	gravity = 10.0f;
};

