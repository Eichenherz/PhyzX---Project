#pragma once

#include "Vec2.h"
#include "PX_Math.h"

//======================================================================//
//																		//
//						PHYSICS DATA STRUCTURES							//
//																		//
//======================================================================//

struct PX_Pose_Data
{
	IVec2			pos;
	Radians			orientation;

	PX_Pose_Data( const IVec2& pos, Radians dgs )
		:
		pos { pos },
		orientation { dgs }
	{}

};

namespace PX
{
	struct Mass_Data
	{
		IVec2	center; // of mass
		float	mass;
		float	I; // @ CM

		Mass_Data( float mass, int side, const IVec2& pos )
			:
			mass { mass },
			center { pos.x + side / 2, pos.y + side / 2 },
			I { 1.0f / 6.0f * mass * float( side * side ) }
		{}
	};

	struct Kinetic_Data
	{
		FVec2	linear_vel;
		float	angular_vel;

		Kinetic_Data( const FVec2& lvel, float avel )
			:
			linear_vel { lvel },
			angular_vel { avel }
		{}
	};

	struct Dynamic_Data
	{
		FVec2	force;
		float	torque;
	};

	//======================================================================//
	//																		//
	//					PHYSICS RIGID BODY SIMULATION 						//
	//																		//
	//======================================================================//
	class Rigid_Body_Physics
	{
	public:
							Rigid_Body_Physics( float mass, int side, const IVec2& mass_ct, PX_Pose_Data& pose0 );

		void				Apply_Force( const FVec2& force, const IVec2& app_pt );
		void				Halt_Force();
		void				Update_Kinetic_State( float dt );
		void				Update_Pose( float dt );

	private:
		//Mabye pointers/references later
		PX_Pose_Data&		pose;
		Mass_Data			mass_data;
		Kinetic_Data		kinetic_state;
		Dynamic_Data		resultant;

		const float			static_Cof;
		const float			kinetic_Cof;

		auto				Linear_Friction() const;
		float				Angular_Friction() const;

		auto				Linear_Accelereation() const;
		float				Angular_Accelereation() const;
	};
}