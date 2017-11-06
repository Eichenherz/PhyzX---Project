#include "PX_Physical_Traits.h"
#include <assert.h>

//===================================================//
//													 //
//					CONSTANTS						 //
//												     //
//===================================================//
static constexpr float	kinetic_friction = 0.15f;
static constexpr float	static_friction = 0.25f;
static constexpr float	gravitational_const = 10.0f;

//===================================================//
//													 //
//			METHODS of PX_Rigid_Body_Physics		 //
//												     //
//===================================================//
PX_Rigid_Body_Physics::PX_Rigid_Body_Physics( float mass, int side, const IVec2& pos )
	:
	mass_data		{ mass, side, pos },
	kinetic_state	{ /* Defaulted to no movement */ },
	f_total			{ { 0.0f,0.0f }, { 0,0 } },
	t_total			{ { { 0.0f,0.0f }, { 0,0 } }, mass_data.mass_center },
	static_drag		{ static_friction * mass_data.mass * gravitational_const },
	kinetic_drag	{ kinetic_friction * mass_data.mass * gravitational_const }
{}

auto PX_Rigid_Body_Physics::Linear_Accelereation()
{
	/* Frictional force with direction opposed to that of movement. */
	auto f_friction = f_total.force.GetNormalized().Negate() * kinetic_drag;

	return ( f_total.force - f_friction ) / mass_data.mass;
}

auto PX_Rigid_Body_Physics::Angular_Accelereation() // may add torque static & kinetic drag consts
{
	/* Frictional torque with direction of rotation opposed to that of rotation. */
	float trq_friction = 0.0f;
	//"Negate" t_total.Normalize() Ouch!!! Remember this is still pseudocode.
	if ( t_total.clockwise_flag() )
	{
		trq_friction = -( kinetic_friction * mass_data.I_cm * gravitational_const );
	}
	else
	{
		trq_friction = kinetic_friction * mass_data.I_cm * gravitational_const;
	}

	return ( t_total.trq - trq_friction ) / mass_data.I_cm;
}

void PX_Rigid_Body_Physics::Apply_Force( const PX_Force & force )
{
	//Defeat the static friction first.
	if ( force.force.GetLength() > static_drag * static_drag  && kinetic_state.linear_vel.GetLength() == 0.0f )
	{
		f_total.force += force.force;
	}
	//And the static torque friction.
	auto torque = Perp_Dot_Prod( mass_data.mass_center - force.app_point, force.force ); // Ouch, may be changed later
	if ( std::fabs( torque ) > static_friction * mass_data.I_cm * gravitational_const && kinetic_state.angular_vel == 0.0f )
	{
		t_total.trq += torque;
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
		f_total.force.Normalize().Negate() *= kinetic_drag;
	}
	//If no movement reset force. But still check for torque.
	else f_total.Make_NULL();

	//***
	//Nor torque.
	if ( kinetic_state.angular_vel != 0.0f )
	{
		//**
		//Same for torque. No perpetuum mobile for you !
		if ( t_total.clockwise_flag() )
		{
			t_total.trq = -( kinetic_friction * mass_data.I_cm * gravitational_const );
		}
		else
		{
			t_total.trq = kinetic_friction * mass_data.I_cm * gravitational_const;
		}
	}
	else t_total.Make_NULL();
	
}

void PX_Rigid_Body_Physics::Update_Kinetic_State( float dt )
{
	//Euler integrate accl to get vel.
	kinetic_state.linear_vel = kinetic_state.linear_vel + Linear_Accelereation() * dt;
	kinetic_state.angular_vel = kinetic_state.angular_vel + Angular_Accelereation() * dt;
}
