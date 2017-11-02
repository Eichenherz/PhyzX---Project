#pragma once

#include "Vec2.h"
#include <vector>
#include "PX_Math.h"
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
		I_cm		{ 1.0f/6.0f * mass * float( side * side )  }
	{}
};

struct PX_Kinetic_Data
{
	IVec2			pos;
	Angle_Degrees	orientation;
	FVec2			linear_vel;
	float			angular_vel;

	PX_Kinetic_Data( const IVec2& pos, Angle_Degrees dgs, const FVec2& lvel = { 0.0f, 0.0f }, float avel = 0.0f )
		:
		pos				{ pos },
		orientation		{ dgs },
		linear_vel		{ lvel },
		angular_vel		{ avel }
	{}
};

struct PX_Force
{
	FVec2 force;
	IVec2 app_point;

	PX_Force( const FVec2& f, const IVec2& app_point )
		:
		force		{ f },
		app_point	{ app_point }
	{}
};


class PX_Rigid_Body_Physics
{
public:
			PX_Rigid_Body_Physics( float mass, int side, const IVec2& pos, Angle_Degrees dgs );

	void	Apply_Force( const PX_Force& force );
	void	Update_Kinetic_State( float dt );
	// Expose data to translate & rotate the box -> PX_Box_Shape to handle geometry & drawing

private:
	PX_Mass_Data			mass_data;
	PX_Kinetic_Data			kinetic_state;
	std::vector<PX_Force>	forces;

	auto Compute_Linear_Accelereation();
	auto Compute_Angular_Accelereation();
};