#include "PX_Physical_Traits.h"
#include <assert.h>

//===================================================//
//													 //
//					CONSTANTS						 //
//												     //
//===================================================//
static constexpr float	kinetic_friction = 0.15f;
static constexpr float	static_friction = 0.05f;
static constexpr float	gravitational_const = 10.0f;

//===================================================//
//													 //
//			METHODS of PX_Rigid_Body_Physics		 //
//												     //
//===================================================//
PX_Rigid_Body_Physics::PX_Rigid_Body_Physics( float mass, int side, const IVec2& mass_ct )
	:
	mass_data				{ mass, side, mass_ct },
	kinetic_state			{ { 0.0f, 0.0f }, 0.0f },
	static_linear_drag		{ static_friction * mass_data.mass * gravitational_const },
	kinetic_linear_drag		{ kinetic_friction * mass_data.mass * gravitational_const },
	static_angular_drag		{ static_friction * mass_data.I * gravitational_const },
	kinetic_angular_drag	{ kinetic_friction * mass_data.I * gravitational_const }
{}

inline auto PX_Rigid_Body_Physics::Linear_Drag() const
{
	return resultant.force.GetNormalized().Negate() * kinetic_linear_drag;
}

inline auto PX_Rigid_Body_Physics::Angular_Drag() const
{
	if ( resultant.torque != 0.0f )
	{
		return std::copysign( kinetic_angular_drag, -resultant.torque );
	}
	return 0.0f;
}

void PX_Rigid_Body_Physics::Apply_Force( const FVec2& force, const IVec2& app_pt )
{
	//If moving or defeated static friction apply force.
	if ( kinetic_state.linear_vel.GetLength() != 0.0f ||
		 ( resultant.force + force ).GetLength() > static_linear_drag * static_linear_drag )
	{
		resultant.force += force;
	}
	//Same for torque.
	auto torque = Perp_Dot_Prod( mass_data.center - app_pt, force );
	if ( kinetic_state.angular_vel != 0.0f ||
		 std::fabs( torque + resultant.torque ) > static_angular_drag )
	{
		resultant.torque += torque;
	}
}

void PX_Rigid_Body_Physics::Halt_Force()
{
	//***
	//You would not want to apply drag if there is no movement, wouldn't you ?
	if ( kinetic_state.linear_vel.GetLength() != 0.0f ) 
	{
		//**
		//If total applied force ceases , then the only remaing force is the drag, 
		//which will slow down the body until it ceases to move.
		resultant.force = Linear_Drag();
	}
	//If no movement reset force. But still check for torque.
	else resultant.force = { 0.0f, 0.0f };

	//***
	//Nor torque.
	if ( kinetic_state.angular_vel != 0.0f )
	{
		//**
		//Same for torque. No perpetuum mobile for you !
		resultant.torque = Angular_Drag();
	}
	//Finally, if no rotation reset torque.
	else resultant.torque = 0.0f;	
}

inline auto PX_Rigid_Body_Physics::Linear_Accelereation() const
{
	return ( resultant.force + Linear_Drag() ) / mass_data.mass;
}

inline auto PX_Rigid_Body_Physics::Angular_Accelereation() const
{
	return ( resultant.torque + Angular_Drag() ) / mass_data.I;
}

void PX_Rigid_Body_Physics::Update_Kinetic_State( float dt )
{
	//Euler integrate accl to get vel.
	kinetic_state.linear_vel += Linear_Accelereation() * dt;
	if ( resultant.torque != Angular_Drag()  )
	{
		kinetic_state.angular_vel += Angular_Accelereation() * dt;
	}
}

const PX_Kinetic_Data& PX_Rigid_Body_Physics::Kinetic_Status() const
{
	return kinetic_state;
}
