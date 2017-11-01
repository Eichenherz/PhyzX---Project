#pragma once

#include "Vec2.h"
#include <vector>
//======================================================================//
//																		//
//						PHYSICS SIMULATION DS							//
//																		//
//======================================================================//

struct PX_Mass_Data
{
	float mass;
	IVec2 mass_center;

	float I_cm;

	PX_Mass_Data( float mass, int side, const IVec2& pos )
		:
		mass		{ mass },
		mass_center { pos.x + side/2, pos.y + side/2 },
		I_cm		{ 1.0f/6.0f * mass * float(side * side)  }
	{}
};

struct PX_Kinetic_Data
{
	IVec2 pos;
	FVec2 linear_vel;
	FVec2 angular_vel;
	bool movement_falg;

	PX_Kinetic_Data( const IVec2& pos, bool movement = false,
					 const FVec2& lvel = { 0.0f, 0.0f }, 
					 const FVec2& avel = { 0.0f, 0.0f } )
		:
		pos				{ pos },
		movement_falg	{ movement },
		linear_vel		{ lvel },
		angular_vel		{ avel }
	{}
};

struct PX_Force
{
	FVec2 force;
	IVec2 app_point;

	PX_Force( const FVec2& f = { 0.0f, 0.0f }, const IVec2& app_point = { 0, 0 } )
		:
		force		{ f },
		app_point	{ app_point }
	{}
};


class PX_Box
{
public:
			PX_Box( float mass, int side, const IVec2& pos );

	void	Apply_Force( const PX_Force& force );
	auto	New_Vel();
	auto	New_Pos();

private:
	PX_Mass_Data			mass_data;
	PX_Kinetic_Data			kinetic_state;
	std::vector<PX_Force>	forces;

	auto Compute_Linear_Accelereation();
	auto Compute_Angular_Accelereation();
};