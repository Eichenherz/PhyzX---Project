#pragma once

#include "Animation.h"
#include "Vec2.h"

class Character
{
private:
	enum class Sequence
	{
		WalkingLeft,
		WalkingRight,
		WalkingUp,
		WalkingDown,
		StandingLeft,
		StandingRight,
		StandingUp,
		StandingDown,
		Count
	};
public:
	Character( const FVec2& pos );
	void Draw( Graphics& gfx ) const;
	void SetDirection( const FVec2& dir );
	void Update( float dt );
	// activates a damage visual effect
	void ActivateEffect();
private:
	Surface sprite;
	FVec2 pos;
	FVec2 vel = { 0.0f,0.0f };
	std::vector<Animation> animations;
	Sequence iCurSequence = Sequence::StandingDown;
	float speed = 110.0f;
	static constexpr float effectDuration = 0.045f;
	float effectTime = 0.0f;
	bool effectActive = false;
};