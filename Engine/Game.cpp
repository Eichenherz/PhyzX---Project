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
	wnd				( wnd ),
	gfx				( wnd ),
	screen_center	{ gfx.GetScreenRect().GetCenter() },
	pose			{ screen_center, angleA },
	poseB			{ screen_center + IVec2 { box_side -20 , - box_side + 30 }, angleB },
	box				{ pose.pos, box_side, box_side },
	boxB			{ poseB.pos, box_side, box_side },
	debug_text		{ "Images\\Fixedsys16x28.bmp" },
	m				{ box.OBB, boxB.OBB }
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
	if ( wnd.kbd.KeyIsPressed( VK_SPACE ) && !flag )
	{
		switch_box = true;
		flag = true;
	} 
	else if ( wnd.kbd.KeyIsPressed( VK_RETURN ) && !flag )
	{
		switch_box = false;
		flag = true;
	}

	if ( wnd.mouse.LeftIsPressed() && !flag )
	{
		if ( !switch_box )
		{
			angleA += Radians { 0.10f };
		} else {
			angleB += Radians { 0.10f };
		}
		flag = true;
	}
	if ( wnd.mouse.RightIsPressed() && !flag )
	{
		if ( !switch_box )
		{
			angleA += Radians { -0.10f };
		} else {
			angleB += Radians { -0.10f };
		}
		flag = true;
	}
	else flag = false; // allows rotation & translation on the same call of UpdateModel. kind of


	IVec2 delta_pos { 0,0 };
	if ( wnd.kbd.KeyIsPressed( VK_UP ) && !flag )
	{
		delta_pos += { 0, -1 };
		flag = true;
	}
	else if ( wnd.kbd.KeyIsPressed( VK_DOWN ) && !flag )
	{
		delta_pos += { 0, 1 };
		flag = true;
	}
	else if ( wnd.kbd.KeyIsPressed( VK_LEFT ) && !flag )
	{
		delta_pos += { -1, 0 };
		flag = true;
	}
	else if ( wnd.kbd.KeyIsPressed( VK_RIGHT ) && !flag )
	{
		delta_pos += { 1, 0 };
		flag = true;
	}
	else flag = false;

	if ( !switch_box )
	{
		posA = delta_pos;
	} else {
		posB = delta_pos;
	}

	box.Transform( PX_Pose_Data { posA, angleA } );
	boxB.Transform( PX_Pose_Data { posB, angleB } );
	

	if ( OBB_Intersection( box.OBB, boxB.OBB ) )
	{
		collision = true;
	}
	else collision = false;

	SAT_Narrowphase( m, box.OBB, boxB.OBB );
}

void Game::ComposeFrame()
{
	box.Draw( gfx, Colors::Blue );
	boxB.Draw( gfx, Colors::Red );

	//Debug
	gfx.Draw_Line( box.Center(), boxB.Center(), Colors::Green );
	if ( collision )
	{
		debug_text.DrawText( "Colliding!", { 10,10 }, Colors::Blue, gfx );
		m.Min_Sep_Axis_Debug( gfx, debug_text );
		m.Debug_Draw( gfx );
	}
}
