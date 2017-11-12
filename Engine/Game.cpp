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
	wnd( wnd ),
	gfx( wnd ),
	rot_test{ gfx.GetScreenRect().GetCenter(), 20}
{
}

void Game::Go()
{
	gfx.BeginFrame();

	/*
	float elapsed_time = ft.Mark();
	while ( elapsed_time > 0.0f )
	{
	const float dt = std::min( euler_h, elapsed_time );


	elapsed_time -= dt;

	}
	*/
	UpdateModel();
	ComposeFrame();
	gfx.EndFrame();
}

void Game::UpdateModel(  )
{
	if( wnd.kbd.KeyIsPressed( VK_LEFT ) )
	{
		angle += Radians { -0.25f };
	}
	if( wnd.kbd.KeyIsPressed( VK_RIGHT ) )
	{
		angle += Radians { 0.25f };
	}
	
	rot_test.Apply_Rotation( angle );
}

void Game::ComposeFrame()
{
	rot_test.Draw( gfx, Colors::Red );
}
