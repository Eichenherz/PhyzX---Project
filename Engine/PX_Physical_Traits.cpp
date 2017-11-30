#include "PX_Physical_Traits.h"
#include <assert.h>
#include "Graphics.h"

//===================================================//
//													 //
//					  CONSTANTS						 //
//												     //
//===================================================//
static constexpr float	kinetic_friction = 0.10f;
static constexpr float	static_friction = 0.25f;
static constexpr float	gravitational_const = 10.0f;
static constexpr float	LINEAR_THRESHOLD = 0.1f;
static constexpr float	ANGULAR_THRESHOLD = 0.5f;

//===================================================//
//													 //
//			METHODS of PX_Rigid_Body_Physics		 //
//												     //
//===================================================//
PX_Rigid_Body_Physics::PX_Rigid_Body_Physics( float mass, int side, const IVec2& mass_ct, PX_Pose_Data& pose )
	:
	mass_data					{ mass, side, mass_ct },
	kinetic_state				{ { 0.0f, 0.0f }, 0.0f },
	pose_data					{ pose },
	static_linear_friction		{ static_friction * mass_data.mass * gravitational_const },
	kinetic_linear_friction		{ kinetic_friction * mass_data.mass * gravitational_const },
	static_angular_friction		{ static_friction * mass_data.I * gravitational_const },
	kinetic_angular_friction	{ kinetic_friction * mass_data.I * gravitational_const }
{
	resultant.force = FVec2 { 0.0f,0.0f };
	resultant.torque = 0.0f;
}

inline auto PX_Rigid_Body_Physics::Linear_Friction() const
{
	return -resultant.force.GetNormalized() * kinetic_linear_friction;
}

inline float PX_Rigid_Body_Physics::Angular_Friction() const
{
	if ( kinetic_state.angular_vel != 0.0f )
	{
		return std::copysign( kinetic_angular_friction, -kinetic_state.angular_vel );
	}
	return 0.0f;
}

void PX_Rigid_Body_Physics::Apply_Force( const FVec2& force, const IVec2& app_pt, Graphics& gfx )
{
	//Apply all forces as long as they are active.
	resultant.force += force;
	resultant.torque += Perp_Dot_Prod( mass_data.center - app_pt, force );
	
	//Debug Draw
	//gfx.Draw_Line( mass_data.center, app_pt, Colors::Green );
	gfx.Draw_Line( app_pt, IVec2( force ) + app_pt, Colors::Red );
}

void PX_Rigid_Body_Physics::Halt_Force()
{
	resultant.force = FVec2 { 0.0f,0.0f };
	resultant.torque = 0.0f;
}

inline auto PX_Rigid_Body_Physics::Linear_Accelereation() const
{
	return ( resultant.force + Linear_Friction() ) / mass_data.mass;
}

inline float PX_Rigid_Body_Physics::Angular_Acceleration() const
{
	return ( resultant.torque + Angular_Friction() ) / mass_data.I;
}

void PX_Rigid_Body_Physics::Update_Kinetic_State( float dt )
{
	//If moving or defeated static friction Euler integrate accl to get vel.
	if ( kinetic_state.linear_vel.GetLength() > LINEAR_THRESHOLD ||
		 resultant.force.GetLength() > static_linear_friction * static_linear_friction )
	{
		kinetic_state.linear_vel += Linear_Accelereation() * dt;
	}
	else
	{
		kinetic_state.linear_vel = FVec2 { 0.0f, 0.0f };
	}

	if ( std::fabs( kinetic_state.angular_vel ) > ANGULAR_THRESHOLD ||
		 std::fabs( resultant.torque ) > static_angular_friction )
	{
		kinetic_state.angular_vel += Angular_Acceleration() * dt;
		//kinetic_state.angular_vel *= 1.0f - dt;
	}
	else
	{
		kinetic_state.angular_vel = 0.0f;
	}
}

const PX_Kinetic_Data& PX_Rigid_Body_Physics::Kinetic_Status() const
{
	return kinetic_state;
}

PX_Kinetic_Data & PX_Rigid_Body_Physics::Kinetic_Status()
{
	return kinetic_state;
}
