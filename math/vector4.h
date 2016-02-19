/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#if !defined(_Vector4_H_)
#define _Vector4_H_

//===================================================
//					DEFINITIONS
//===================================================


//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include <limits>
#include <cmath>
#include <mymath.h>

//===================================================
//			CLASS DESCRIPTION:
//			    a templated 3d homogeneous vector  
//				class, mainly for use with float
//				and double data type
//===================================================

namespace i3d {

/**
* @brief A class for a 4d vector and various 4d vector operations
*
* A class for a 4d vector and various 4d vector operations
*/    
template<class T>
class Vector4 {

public:
	/* constructor */
	Vector4(T a, T b, T c, T d): x(a), y(b), z(c), w(d) {}

	Vector4(T a, T b, T c): x(a), y(b), z(c), w(1) {}

	/* copy constructor */
	Vector4(const Vector4 &v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		w = v.w;
	}

    /* default constructor */
	Vector4():x(0), y(0), z(0), w(1){}
	~Vector4(){};

//===================================================
//  			Assignment operator		
//===================================================
	
	inline const Vector4& operator=(const Vector4& v)
	{
		
		x = v.x;
		y = v.y;
		z = v.z;
		w = v.w;
		
		return *this;
	}//end  operator
	
	inline Vector4 operator+(const Vector4 &v) const
	{
		return Vector4(x + v.x, y + v.y, z + v.z, 1);
	}//end  operator

	inline Vector4 operator-(const Vector4 &v) const
	{
		return Vector4(x - v.x, y - v.y, z - v.z, 1);
	}//end  operator

	inline Vector4 operator - () const
	{
		return Vector4(-x,-y,-z);
	}//end operator

	inline Vector4 operator*(T num) const
	{
		// Return scaled vector
		return Vector4(x * num, y * num, z * num);
	}//end  operator

	inline T operator * (const Vector4 &rhs) const
	{
		return  x * rhs.x +
				y * rhs.y +
				z * rhs.z;
				
	}//end  operator

	inline T norm2()
	{
		return  (x * x) + (y * y) + (z * z);
	}//end  operator

	inline T mag()
	{
		return std::sqrt(norm2());
	}//end  operator

	inline static Vector4 createVector(const Vector4 &a, const Vector4 &b) 
	{
		Vector4 res = b - a;
		return res;
	}//end  operator

	inline const Vector4& operator /= (const T &rhs)
	{
		x /= rhs;
		y /= rhs;
		z /= rhs;
		return *this;
	}//end  operator

	inline const Vector4& operator += (const Vector4 &rhs)
	{

		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		return *this;
	}//end  operator

	inline const Vector4& operator -= (const Vector4 &rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		return *this;
	}//end  operator
	
	inline const Vector4& operator *= (const T &d)
	{
		x *= d;
		y *= d;
		z *= d;
		
		return *this;
	}//end  operator

	inline T dot(const Vector4 &a, const Vector4 &b)
	{
		return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
	}//end  operator

		
	inline static Vector4 Cross(Vector4 vVector1, Vector4 vVector2)
	{

		Vector4 vCross;

		vCross.x = ((vVector1.y * vVector2.z) - (vVector1.z * vVector2.y));
		vCross.y = ((vVector1.z * vVector2.x) - (vVector1.x * vVector2.z));
		vCross.z = ((vVector1.x * vVector2.y) - (vVector1.y * vVector2.x));

		return vCross;

	}//end Cross

	inline void Normalize()
	{
		T dInvMag = 1.0/mag();
		x *= dInvMag;
		y *= dInvMag;
		z *= dInvMag;
		w  = 1;
	}//end Normalize

	inline int FindMaxAbsComponent () const
	{

		T max = -std::numeric_limits<T>::max();
		int iResult = -1;

		for(int i = 0; i < 3; i++)
		{

			T absVal = std::abs(m_dCoords[i]);

			if(absVal > max)
			{
				max = absVal;
				iResult = i;
			}//end if

		}//end for

		return iResult;

	}//end FindMaxAbsComponent
	
	template <typename Templateparm>
	friend std::ostream& operator<<(std::ostream& out, const Vector4<Templateparm>& v1); 
	
	/* union to allow different access methods */
	union
	{
		T m_dCoords[4];

		struct
		{
			T x;
			T y;
		    T z;
			T w;
		};
	};

};

//template<class T> Vector3<T> operator*(T a,const Vector3<T> &vRHS);
template<typename T> Vector4<T> operator*(T num, const Vector4<T> &vRHS);

template<typename T>
Vector4<T> operator*(T num, const Vector4<T> &vRHS) 
{
	// Return scaled vector
	return Vector4<T>(vRHS.x * num, vRHS.y * num, vRHS.z * num);
}//end  operator

/* typedefs to create float and double vectors */
typedef Vector4<double> Vector4d;
typedef Vector4<float>  Vector4f;
typedef Vector4<Real>  Vec4;

}

#endif  //_Vector4_H_
