#pragma once

#include "Vec2.h"
#include "PX_Math.h"

//======================================================================//
//																		//
//						PHYSICS DATA STRUCTURES							//
//																		//
//======================================================================//
struct PX_Mass_Data
{
	IVec2	center; // of mass
	float	mass;
	float	I; // @ CM

	PX_Mass_Data( float mass, int side, const IVec2& pos )
		:
		mass	{ mass },
		center  { pos.x + side/2, pos.y + side/2 },
		I		{ 1.0f/6.0f * mass * float( side * side )  }
	{}
};

struct PX_Pose_Data
{
	IVec2			pos;
	Radians			orientation;

	PX_Pose_Data( const IVec2& pos, Radians dgs )
		:
		pos				{ pos },
		orientation		{ dgs }
	{}
};

struct PX_Kinetic_Data
{
	FVec2	linear_vel;
	float	angular_vel;

	PX_Kinetic_Data( const FVec2& lvel,  float avel )
		:
		linear_vel		{ lvel },
		angular_vel		{ avel }
	{}
};

struct PX_Dynamic_Data
{
	FVec2	force;
	float	torque;
};

//======================================================================//
//																		//
//					PHYSICS RIGID BODY SIMULATION 						//
//																		//
//======================================================================//
class PX_Rigid_Body_Physics
{
public:
							PX_Rigid_Body_Physics( float mass, int side, const IVec2& pos );

	void					Apply_Force( const FVec2& force, const IVec2& app_pt );
	void					Halt_Force();
	void					Update_Kinetic_State( float dt );
	const PX_Kinetic_Data&	Kinetic_Status() const;

private:
	PX_Mass_Data			mass_data;
	PX_Kinetic_Data			kinetic_state;
	PX_Dynamic_Data			resultant;

	const float				static_linear_drag;
	const float				kinetic_linear_drag;
	const float				static_angular_drag;
	const float				kinetic_angular_drag;

	auto					Linear_Drag() const;
	auto					Angular_Drag() const;

	auto					Linear_Accelereation() const;
	auto					Angular_Accelereation() const;
};