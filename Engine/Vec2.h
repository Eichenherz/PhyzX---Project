#pragma once

#include <cmath>

enum class Vec_Type
{
	COLUMN,
	ROW
};

template<typename T, Vec_Type t = Vec_Type::COLUMN>
class Vec2_
{
public:
			Vec2_() = default;
			Vec2_( T x_in,T y_in )
		:
		x( x_in ),
		y( y_in )
	{}
	template<typename S>
	explicit Vec2_( const Vec2_<S>& src )
		:
		x( (T)src.x ),
		y( (T)src.y )
	{}
	Vec2_	operator+( const Vec2_& rhs ) const
	{
		return Vec2_( *this ) += rhs;
	}
	Vec2_&	operator+=( const Vec2_& rhs )
	{
		x += rhs.x;
		y += rhs.y;
		return *this;
	}
	Vec2_	operator-( const Vec2_& rhs ) const
	{
		return Vec2_( *this ) -= rhs;
	}
	Vec2_&	operator-=( const Vec2_& rhs )
	{
		x -= rhs.x;
		y -= rhs.y;
		return *this;
	}
	Vec2_	operator*( T rhs ) const
	{
		return Vec2_( *this ) *= rhs;
	}
	Vec2_&	operator*=( T rhs )
	{
		x *= rhs;
		y *= rhs;
		return *this;
	}
	Vec2_	operator/( T rhs ) const
	{
		return Vec2_( *this ) /= rhs;
	}
	Vec2_&	operator/=( T rhs )
	{
		x /= rhs;
		y /= rhs;
		return *this;
	}
	Vec2_	operator-() const
	{
		return Vec2_( -x,-y );
	}
	T		GetLength() const
	{
		return (T)std::sqrt( GetLengthSq() );
	}
	T		GetLengthSq() const
	{
		return x * x + y * y;
	}
	Vec2_&	Normalize()
	{
		return *this = GetNormalized();
	}
	Vec2_	GetNormalized() const
	{
		const T len = GetLength();
		if( len != (T)0 )
		{
			return *this / len;
		}
		return *this;
	}
	Vec2_	GetPerp() const
	{
		return { -this->y, this->x };
	}
	Vec2_&	Negate()
	{
		this->x = -x;
		this->y = -y;

		return *this;
	}

	bool	is_Column() const
	{
		return t == Vec_Type::COLUMN;
	}
public:
	T x;
	T y;
};

using FVec2 = Vec2_<float>;
using IVec2 = Vec2_<int>;


template<class Vec>
float Dot_Prod( const Vec& vec1, const Vec& vec2 )
{
	return vec1.x * vec2.x + vec1.y * vec2.y;
}

template<class IVec, class FVec>
FVec2 Cross_Prod( const IVec& vec1, const FVec& vec2 )
{
}

template<class IVec, class FVec>
float Perp_Dot_Prod( const IVec& vec1, const FVec& vec2 )
{
	auto perp_v1 = vec1.GetPerp();

	return perp_v1.x * vec2.x + perp_v1.y * vec2.y;
}