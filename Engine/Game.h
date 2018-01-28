/****************************************************************************************** 
 *	Chili DirectX Framework Version 16.07.20											  *	
 *	Game.h																				  *
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
#pragma once

#include "Keyboard.h"
#include "Mouse.h"
#include "Graphics.h"
#include "Timer.h"
#include "Sound.h"
#include "PX_Box_Shape.h"
#include "PX_Physical_Traits.h"
#include "Font.h"
#include "PX_Collision.h"

class Game
{
public:
	Game( class MainWindow& wnd );
	Game( const Game& ) = delete;
	Game& operator=( const Game& ) = delete;
	void Go();

private:
	void ComposeFrame();
	void UpdateModel( float dt );
	/********************************/
	/*  User Functions              */
	/********************************/

private:
	MainWindow&				wnd;
	Graphics				gfx;
	/********************************/
	/*  User Variables              */
	static constexpr float	euler_h = 0.015f;
	const IVec2				screen_center;
	Timer					ft;

	static constexpr int	box_side = 50;
	static constexpr float	mass = 5.0f;
	Radians					angleA = 0.0f;
	Radians					angleB = 0.785398163f;
	IVec2					posA;
	IVec2					posB;

	PX_Pose_Data			pose;
	PX_Pose_Data			poseB;
	PX_Box_Shape			box;
	PX_Box_Shape			boxB;

	IVec2					pos;

	bool					flag = false;
	bool					collision = false;
	Font					debug_text;

	Manifold				m;
	/********************************/
};