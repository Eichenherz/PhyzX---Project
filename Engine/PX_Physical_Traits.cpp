#include "PX_Physical_Traits.h"
#include <assert.h>

//===================================================//
//													 //
//					  CONSTANTS						 //
//												     //
//===================================================//
static constexpr float	kinetic_friction = 0.20f;
static constexpr float	static_friction = 0.35f;
static constexpr float	gravitational_const = 10.0f;
static constexpr float	LINEAR_MOV_THRESHOLD = 0.1f;
static constexpr float	ANGULAR_MOV_THRESHOLD = 0.01f;

//===================================================//
//													 //
//			METHODS of PX_Rigid_Body_Physics		 //
//												     //
//===================================================//
PX_Rigid_Body_Physics::PX_Rigid_Body_Physics( float mass, int side, const IVec2& mass_ct )
	:
	mass_data					{ mass, side, mass_ct },
	kinetic_state				{ { 0.0f, 0.0f }, 0.0f },
	static_linear_friction		{ static_friction * mass_data.mass * gravitational_const },
	kinetic_linear_friction		{ kinetic_friction * mass_data.mass * gravitational_const },
	static_angular_friction		{ static_friction * mass_data.I * gravitational_const },
	kinetic_angular_friction	{ kinetic_friction * mass_data.I * gravitational_const }
{}

inline auto PX_Rigid_Body_Physics::Linear_Drag() const
{
	return kinetic_linear_friction;
}

inline auto PX_Rigid_Body_Physics::Angular_Drag() const
{
	return 0.0f;
}

void PX_Rigid_Body_Physics::Apply_Force( const FVec2& force, const IVec2& app_pt )
{
	//Apply all forces as long as they are active.
	resultant.force += force;
	resultant.torque += Perp_Dot_Prod( mass_data.center - app_pt, force );
	//( mass_data.center - app_pt ).x * force.y - ( mass_data.center - app_pt ).y * force.x;
}

void PX_Rigid_Body_Physics::Halt_Force()
{
	resultant.force = FVec2 { 0.0f,0.0f };
	resultant.torque = 0.0f;
}

inline auto PX_Rigid_Body_Physics::Linear_Accelereation() const
{
	return resultant.force / mass_data.mass;
}

inline auto PX_Rigid_Body_Physics::Angular_Accelereation() const
{
	return resultant.torque / mass_data.I;
}

void PX_Rigid_Body_Physics::Update_Kinetic_State( float dt )
{
	//If moving or defeated static friction Euler integrate accl to get vel.
	if ( kinetic_state.linear_vel.GetLength() != 0.0f ||
		 resultant.force.GetLength() > static_linear_friction * static_linear_friction )
	{
		kinetic_state.linear_vel += Linear_Accelereation() * dt;
		kinetic_state.linear_vel *= 1.0f - kinetic_friction / 2.0f;
	}
	else if ( kinetic_state.linear_vel.GetLength() > LINEAR_MOV_THRESHOLD )
	{
		kinetic_state.linear_vel *= 1.0f - kinetic_friction / 2.0f;
	}

	if ( kinetic_state.angular_vel != 0.0f ||
		 std::fabs( resultant.torque ) > static_angular_friction )
	{
		kinetic_state.angular_vel += Angular_Accelereation() * dt;
		kinetic_state.angular_vel *= 1.0f - kinetic_friction / 10.0f;
	}
	else if ( kinetic_state.angular_vel > ANGULAR_MOV_THRESHOLD )
	{
		kinetic_state.angular_vel *= 1.0f - kinetic_friction / 10.0f;
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
