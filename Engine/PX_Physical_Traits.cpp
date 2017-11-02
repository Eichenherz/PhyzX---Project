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
PX_Rigid_Body_Physics::PX_Rigid_Body_Physics( float mass, int side, const IVec2& pos, Angle_Degrees dgs )
	:
	mass_data		{ mass, side, pos },
	kinetic_state	{ pos, dgs }
{}

void PX_Rigid_Body_Physics::Apply_Force( const PX_Force & force )
{
	const float static_friction_force = static_friction * mass_data.mass * gravitational_const;
	if ( force.force.GetLength() > static_friction_force * static_friction_force )
	{
		forces.emplace_back( force );
	}
}

auto PX_Rigid_Body_Physics::Compute_Linear_Accelereation()
{
	FVec2 F_total = { 0.0f, 0.0f };
	for ( const auto& f : forces ) F_total += f.force;

	/* Frictional force with direction opposed to that of movement. */
	F_total = F_total.GetNormalized().Negate() * kinetic_friction * mass_data.mass * gravitational_const;

	return F_total / mass_data.mass;
}

auto PX_Rigid_Body_Physics::Compute_Angular_Accelereation()
{
	float Trq_total = 0.0f;
	for ( const auto& f : forces )
	{
		const IVec2 r = mass_data.mass_center - f.app_point;
		Trq_total += Perp_Dot_Prod( r, f.force );
	}

	/* Frictional torque with direction opposed to that of rotation. */
	if ( std::signbit( Trq_total ) )
	{
		Trq_total -= kinetic_friction * mass_data.I_cm * gravitational_const;
	}
	else
	{
		Trq_total += kinetic_friction * mass_data.I_cm * gravitational_const;
	}

	return Trq_total / mass_data.I_cm;
}
