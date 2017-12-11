#include "PX_Physical_Traits.h"
#include <assert.h>

using namespace PX;
//===================================================//
//													 //
//					  CONSTANTS						 //
//												     //
//===================================================//
static constexpr float	kinetic_friction = 0.15f;
static constexpr float	static_friction = 0.35f;
static constexpr float	gravitational_const = 10.0f;
static constexpr float	LINEAR_THRESHOLD = 0.25f;
static constexpr float	ANGULAR_THRESHOLD = 0.5f;

//===================================================//
//													 //
//			METHODS of PX_Rigid_Body_Physics		 //
//												     //
//===================================================//
Rigid_Body_Physics::Rigid_Body_Physics( float mass, int side, const IVec2& mass_ct, PX_Pose_Data& pose0 )
	:
	pose			{ pose0 },
	mass_data		{ mass, side, mass_ct },
	kinetic_state	{ { 0.0f, 0.0f }, 0.0f }, 
	static_Cof		{ static_friction },
	kinetic_Cof		{ kinetic_friction }
{
	resultant.force = FVec2 { 0.0f,0.0f };
	resultant.torque = 0.0f;
}

inline auto Rigid_Body_Physics::Linear_Friction() const
{
	return resultant.force.GetNormalized().Negate() * kinetic_Cof * mass_data.mass * gravitational_const;
}

inline float Rigid_Body_Physics::Angular_Friction() const
{
	if ( kinetic_state.angular_vel != 0.0f )
	{
		return std::copysign( kinetic_Cof * mass_data.I * gravitational_const, -kinetic_state.angular_vel );
	}
	return 0.0f;
}

void Rigid_Body_Physics::Apply_Force( const FVec2& force, const IVec2& app_pt )
{
	resultant.force += force;
	resultant.torque += Perp_Dot_Prod( mass_data.center - app_pt, force );
}

void Rigid_Body_Physics::Halt_Force()
{
	resultant.force = FVec2 { 0.0f,0.0f };
	resultant.torque = 0.0f;
}

inline auto Rigid_Body_Physics::Linear_Accelereation() const
{
	return ( resultant.force  + Linear_Friction() ) / mass_data.mass;
}

inline float Rigid_Body_Physics::Angular_Accelereation() const
{
	return ( resultant.torque + Angular_Friction() ) / mass_data.I;
}

void Rigid_Body_Physics::Update_Kinetic_State( float dt )// static_Cof must be static_friction
{
	//If moving or defeated static friction Euler integrate accl to get vel.
	if ( kinetic_state.linear_vel.GetLength() > LINEAR_THRESHOLD ||
		 resultant.force.GetLength() > static_Cof * static_Cof * mass_data.mass * gravitational_const  * mass_data.mass * gravitational_const )//
	{
		kinetic_state.linear_vel += Linear_Accelereation() * dt;
	}
	else
	{
		kinetic_state.linear_vel = FVec2 { 0.0f,0.0f };
	}

	if ( std::fabs( kinetic_state.angular_vel ) > ANGULAR_THRESHOLD ||
		 std::fabs( resultant.torque ) > static_Cof * mass_data.mass * gravitational_const )//
	{
		kinetic_state.angular_vel += Angular_Accelereation() * dt;
	}
	else
	{
		kinetic_state.angular_vel = 0.0f;
	}
}

void Rigid_Body_Physics::Update_Pose( float dt )
{
	pose.pos = IVec2( kinetic_state.linear_vel * dt ); // Must not be += . At least yet.
	pose.orientation += kinetic_state.angular_vel * dt;
}

