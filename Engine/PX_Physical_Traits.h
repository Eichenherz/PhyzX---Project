#pragma once

#include "Vec2.h"
#include "PX_Math.h"

//======================================================================//
//																		//
//						PHYSICS DATA STRUCTURES							//
//																		//
//======================================================================//
struct PX_Mass_Data // Mass_Properties
{
	IVec2	center; // of mass
	float	mass;
	float	I; // @ CM

	PX_Mass_Data( float mass, int side, const IVec2& pos )
		:
		mass	{ mass },
		center  { pos.x + side / 2, pos.y + side / 2 },
		I		{ 1.0f/6.0f * mass * float( side * side )  }
	{}
};

struct PX_Pose_Data
{
	IVec2			pos;
	//Radians			orientation;
	float	orientation;

	PX_Pose_Data( const IVec2& pos, float dgs )
		:
		pos				{ pos },
		orientation		{ dgs }
	{}
	PX_Pose_Data( const PX_Pose_Data& p )
		: 
		pos { p.pos },
		orientation { p.orientation }
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
							PX_Rigid_Body_Physics( float mass, int side, const IVec2& mass_ct, PX_Pose_Data&  pose );

	void					Apply_Force( const FVec2& force, const IVec2& app_pt, class Graphics& gfx );
	void					Halt_Force();
	void					Update_Kinetic_State( float dt );
	const PX_Kinetic_Data&	Kinetic_Status() const;
	PX_Kinetic_Data&		Kinetic_Status();
	//void					Debug_Draw(  ) const;

public://private:
	PX_Mass_Data			mass_data;
	PX_Dynamic_Data			resultant;
	PX_Kinetic_Data			kinetic_state;
	PX_Pose_Data&			pose_data;

	const float				static_linear_friction;
	const float				kinetic_linear_friction;
	const float				static_angular_friction;
	const float				kinetic_angular_friction;

	auto					Linear_Friction() const;
	float					Angular_Friction() const;

	auto					Linear_Accelereation() const;
	float					Angular_Acceleration() const;
};