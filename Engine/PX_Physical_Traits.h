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

struct PX_Pose_Data
{
	IVec2			pos;
	Angle_Degrees	orientation;// Might be changed

	PX_Pose_Data( const IVec2& pos, Angle_Degrees dgs )
		:
		pos				{ pos },
		orientation		{ dgs }
	{}
};

struct PX_Kinetic_Data
{
	
	FVec2	linear_vel;
	float	angular_vel;

	PX_Kinetic_Data( const FVec2& lvel = { 0.0f, 0.0f }, float avel = 0.0f )
		:
		linear_vel		{ lvel },
		angular_vel		{ avel }
	{}
};

struct PX_Force
{
	FVec2	force;
	IVec2	app_point;

	PX_Force( const FVec2& f, const IVec2& app_point )
		:
		force		{ f },
		app_point	{ app_point }
	{}
};

struct PX_Torque // Work in progress
{
	float trq;

	PX_Torque( const PX_Force& f, const IVec2& mass_ct )
		:
		trq		{ Perp_Dot_Prod( mass_ct - f.app_point, f.force ) }
	{}

	bool clockwise_flag() const
	{
		return std::signbit( trq );
	}
};

class PX_Rigid_Body_Physics
{
public:
			PX_Rigid_Body_Physics( float mass, int side, const IVec2& pos );

	void	Apply_Force( const PX_Force& force );
	void	Update_Kinetic_State( float dt );
	// Expose data to translate & rotate the box -> PX_Box_Shape to handle geometry & drawing

private:
	PX_Mass_Data			mass_data;
	PX_Kinetic_Data			kinetic_state;
	PX_Force				f_total;
	PX_Torque				t_total;

	const float				static_drag;
	const float				kinetic_drag;



	auto	Linear_Accelereation();
	auto	Angular_Accelereation();
};