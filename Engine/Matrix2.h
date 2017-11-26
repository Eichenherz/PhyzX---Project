#pragma once

#include "Vec2.h"
#include <cassert>
#include <algorithm>


template<typename T>
class Matrix2
{
public:
				Matrix2() = default;
				Matrix2( T a_11, T a_12, T a_21, T a_22 );

	Matrix2		operator+ ( const Matrix2& rhs ) const;
	Matrix2&	operator+= ( const Matrix2& rhs );
	Matrix2		operator- ( const Matrix2& rhs ) const;
	Matrix2&	operator-= ( const Matrix2& rhs );
	Matrix2		operator*( const Matrix2& rhs ) const;
	Matrix2&	operator*=( const Matrix2& rhs );
	Matrix2		operator* ( T scalar ) const;
	Matrix2&	operator*= ( T scalar );
	Matrix2		operator/ ( T scalar ) const;
	Matrix2&	operator/= ( T scalar );

	template<typename S>
	Vec2_<S>	operator*( const Vec2_<S>& rhs ) const;
	template<typename S>
	Vec2_<S>&	operator*=( Vec2_<S>& rhs ) const;

protected:
	T	a11, a12,
		a21, a22;
};

using FMtrx2 = Matrix2<float>;
using IMtrx2 = Matrix2<int>;

class RotMtrx2 : public FMtrx2
{
public:
				RotMtrx2() : FMtrx2 { 1.0f, 0.0f, 0.0f, 1.0f }
				{}
				RotMtrx2( float theta ) : FMtrx2 { std::cos( theta ), -( std::sin( theta ) ),
												   std::sin( theta ), std::cos( theta ) }
				{}
				RotMtrx2( const RotMtrx2& r ) : FMtrx2 { r.a11, r.a12, r.a21, r.a22 }
				{}
				RotMtrx2( const FMtrx2& f )
					:
					FMtrx2 { f }
				{}


	FVec2		Basis_X() const
	{
		return { a11, a12 };
	}
	FVec2		Basis_Y() const
	{
		return { a21, a22 };
	}
	RotMtrx2&	invert()
	{
		std::swap( a12, a21 );
		return *this;
	}
	RotMtrx2	inverted() const
	{
		return RotMtrx2 { *this }.invert();
	}
};



template<typename T>
Matrix2<T>::Matrix2( T a_11, T a_12, T a_21, T a_22 )
	:
	a11	{ a_11 },
	a12 { a_12 },
	a21	{ a_21 },
	a22 { a_22 }
{}

template<typename T>
Matrix2<T> Matrix2<T>::operator+( const Matrix2 & rhs ) const
{
	return Matrix2( a11 + rhs.a11, a12 + rhs.a12,
					a21 + rhs.a21, a22 + rhs.a22 );
}

template<typename T>
Matrix2<T>& Matrix2<T>::operator+=( const Matrix2 & rhs ) 
{
	this->a11 += rhs.a11;
	this->a12 += rhs.a12;
	this->a21 += rhs.a21;
	this->a22 += rhs.a22;

	return *this;
}

template<typename T>
Matrix2<T> Matrix2<T>::operator-( const Matrix2 & rhs ) const
{
	return Matrix2( a11 - rhs.a11, a12 - rhs.a12,
					a21 - rhs.a21, a22 - rhs.a22 );
}

template<typename T>
Matrix2<T>& Matrix2<T>::operator-=( const Matrix2 & rhs )
{
	this->a11 -= rhs.a11;
	this->a12 -= rhs.a12;
	this->a21 -= rhs.a21;
	this->a22 -= rhs.a22;

	return *this;
}

template<typename T>
Matrix2<T> Matrix2<T>::operator*( const Matrix2 & rhs ) const
{
	return Matrix2( a11 * rhs.a11 + a12 * rhs.a21, a11 * rhs.a12 + a12 * rhs.a22,
					a21 * rhs.a11 + a22 * rhs.a21, a21 * rhs.a12 + a22 * rhs.a22 );
}

template<typename T>
Matrix2<T>& Matrix2<T>::operator*=( const Matrix2 & rhs )
{
	a11 = a11 * rhs.a11 + a12 * rhs.a21;
	a12 = a11 * rhs.a12 + a12 * rhs.a22;
	a21 = a21 * rhs.a11 + a22 * rhs.a21;
	a22 = a21 * rhs.a12 + a22 * rhs.a22;

	return *this;
}

template<typename T>
Matrix2<T> Matrix2<T>::operator*( T scalar ) const
{
	return Matrix2( scalar * a11, scalar * a12,
					scalar * a21, scalar * a22 );
}

template<typename T>
Matrix2<T>& Matrix2<T>::operator*=( T scalar )
{
	this->a11 *= scalar;
	this->a12 *= scalar;
	this->a21 *= scalar;
	this->a22 *= scalar;

	return *this;
}

template<typename T>
Matrix2<T> Matrix2<T>::operator/( T scalar ) const
{
	return Matrix2( a11 / scalar, a12 / scalar,
					a21 / scalar, a22 / scalar );
}

template<typename T>
Matrix2<T>& Matrix2<T>::operator/=( T scalar )
{
	this->a11 /= scalar;
	this->a12 /= scalar;
	this->a21 /= scalar;
	this->a22 /= scalar;

	return *this;
}

template<typename T>
template<typename S>
Vec2_<S> Matrix2<T>::operator*( const Vec2_<S>& rhs ) const
{
	assert( rhs.is_Column() );

	auto rhs_x = rhs.x;
	auto rhs_y = rhs.y;

	return Vec2_<S>( S( a11 * rhs_x + a12 * rhs_y ),
					 S( a21 * rhs_x + a22 * rhs_y ) );
}

template<typename T>
template<typename S>
Vec2_<S>& Matrix2<T>::operator*=( Vec2_<S>& rhs ) const
{
	assert( rhs.is_Column() );

	auto rhs_x = rhs.x;
	auto rhs_y = rhs.y;

	rhs.x = S( a11 * rhs_x + a12 * rhs_y );
	rhs.y = S( a21 * rhs_x + a22 * rhs_y );
	return rhs;
}

	