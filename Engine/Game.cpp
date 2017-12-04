/****************************************************************************************** 
 *	Chili DirectX Framework Version 16.07.20											  *	
 *	Game.cpp																			  *
 *	Copyright 2016 PlanetChili.net <http://www.planetchili.net>							  *
 *																						  *
 *	This file is part of The Chili DirectX Framework.									  *
 *																						  *
 *	The Chili DirectX Framework is free software: you can redistribute it and/or modify	  *
 *	it under the terms of the GNU General Public License as published by				  *
 *	the Free Software Foundation, either version 3 of the License, or					  *
 *	(at your option) any later version.													  *
 *																						  *
 *	The Chili DirectX Framework is distributed in the hope that it will be useful,		  *
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of						  *
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the						  *
 *	GNU General Public License for more details.										  *
 *																						  *
 *	You should have received a copy of the GNU General Public License					  *
 *	along with The Chili DirectX Framework.  If not, see <http://www.gnu.org/licenses/>.  *
 ******************************************************************************************/
#include "MainWindow.h"
#include "Game.h"

Game::Game( MainWindow& wnd )
	:
	wnd		( wnd ),
	gfx		( wnd ),
	pose	{ gfx.GetScreenRect().GetCenter(), 0.0f },
	box		{ pose.pos, box_side, box_side },
	//poseB	{ { 110 + box_side/2, 100 }, angleB },
	//boxB	{ poseB.pos, box_side, box_side },
	phyzx	{ mass, box_side, box.Center(), pose },
	debug_text { "Images\\Fixedsys16x28.bmp" }
{
}

void Game::Go()
{
	gfx.BeginFrame();
	float elapsed_time = ft.Mark();
	while ( elapsed_time > 0.0f )
	{
		const float dt = std::min( euler_h, elapsed_time );
		UpdateModel( dt );

		elapsed_time -= dt;
	}
	ComposeFrame();
	gfx.EndFrame();
}

void Game::UpdateModel( float dt )
{
	FVec2 f { 50.0f, 50.0f }; 
	FVec2 g = -f;

	if ( wnd.mouse.LeftIsPressed() && flag == false )
	{
		phyzx.Apply_Force( f, box.Center() + IVec2 { box_side, box_side / 8 } );
		phyzx.Apply_Force( g, box.Center() + IVec2 { -box_side, -box_side / 8 } );
		flag = true;
	}
	if ( !wnd.mouse.LeftIsPressed() )
	{
		flag = false;
	}

	if ( wnd.mouse.RightIsPressed() )
	{
		phyzx.Halt_Force();
	}
	phyzx.Update_Kinetic_State( dt );
	pose.pos = IVec2( phyzx.Kinetic_Status().linear_vel * dt );
	pose.orientation += phyzx.Kinetic_Status().angular_vel * dt;
	
	box.Transform( pose );
	/*
	if ( wnd.kbd.KeyIsPressed( VK_LEFT ) )  angleA += Radians { 0.15f };
	if ( wnd.kbd.KeyIsPressed( VK_RIGHT ) ) angleB += Radians { -0.15f };

	boxA.Transform( PX_Pose_Data { posA, angleA } );
	boxB.Transform( PX_Pose_Data { posB, angleB } );

	if ( OBB_Intersection( boxA.OBB, boxB.OBB ) )
	{
		hit.Play();
	}
	boxA.Draw( gfx, Colors::Red );
	boxB.Draw( gfx, Colors::Blue );*/
}

void Game::ComposeFrame()
{
	box.Draw( gfx, Colors::Blue );
	debug_text.DrawText( "Angular Vel " + std::to_string( phyzx.Kinetic_Status().angular_vel ), { 10,10 }, Colors::Blue, gfx );
	debug_text.DrawText( "Angular Friction " + std::to_string( phyzx.Angular_Friction() ), { 10,40 }, Colors::Blue, gfx );
	debug_text.DrawText( "Angular Accl " + std::to_string( phyzx.Angular_Acceleration() ), { 10,70 }, Colors::Blue, gfx );
}
