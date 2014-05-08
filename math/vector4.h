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

#if !defined(_CVector4_H_)
#define _CVector4_H_

//===================================================
//					DEFINITIONS
//===================================================


//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include <limits>
#include <cmath>

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
class CVector4 {

public:
	/* constructor */
	CVector4(T a, T b, T c, T d): x(a), y(b), z(c), w(d) {}

	CVector4(T a, T b, T c): x(a), y(b), z(c), w(1) {}

	/* copy constructor */
	CVector4(const CVector4 &v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		w = v.w;
	}

    /* default constructor */
	CVector4():x(0), y(0), z(0), w(1){}
	~CVector4(){};

//===================================================
//  			Assignment operator		
//===================================================
	
	inline const CVector4& operator=(const CVector4& v)
	{
		
		x = v.x;
		y = v.y;
		z = v.z;
		w = v.w;
		
		return *this;
	}//end  operator
	
	inline CVector4 operator+(const CVector4 &v) const
	{
		return CVector4(x + v.x, y + v.y, z + v.z, 1);
	}//end  operator

	inline CVector4 operator-(const CVector4 &v) const
	{
		return CVector4(x - v.x, y - v.y, z - v.z, 1);
	}//end  operator

	inline CVector4 operator - () const
	{
		return CVector4(-x,-y,-z);
	}//end operator

	inline CVector4 operator*(T num) const
	{
		// Return scaled vector
		return CVector4(x * num, y * num, z * num);
	}//end  operator

	inline T operator * (const CVector4 &rhs) const
	{
		return  x * rhs.x +
				y * rhs.y +
				z * rhs.z;
				
	}//end  operator

	inline T norm2()
	{
		return  (x * x) + (y * y) + (z * z);
	}//end  operator

	inline double mag()
	{
		return sqrt(norm2());
	}//end  operator

	inline static CVector4 createVector(const CVector4 &a, const CVector4 &b) 
	{
		CVector4 res = b - a;
		return res;
	}//end  operator


	inline const CVector4& operator /= (const T &rhs)
	{
		x /= rhs;
		y /= rhs;
		z /= rhs;
		return *this;
	}//end  operator


	inline const CVector4& operator += (const CVector4 &rhs)
	{

		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		return *this;
	}//end  operator

	
	inline const CVector4& operator -= (const CVector4 &rhs)
	{

		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		
		return *this;
	}//end  operator

	
	inline const CVector4& operator *= (const T &d)
	{
		x *= d;
		y *= d;
		z *= d;
		
		return *this;
	}//end  operator

	inline T dot(const CVector4 &a, const CVector4 &b)
	{
		return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
	}//end  operator

		
	inline static CVector4 Cross(CVector4 vVector1, CVector4 vVector2)
	{
		CVector4 vCross;

		vCross.x = ((vVector1.y * vVector2.z) - (vVector1.z * vVector2.y));

		vCross.y = ((vVector1.z * vVector2.x) - (vVector1.x * vVector2.z));

		vCross.z = ((vVector1.x * vVector2.y) - (vVector1.y * vVector2.x));

		return vCross;
	}//end Cross

	inline void Normalize()
	{
		double dInvMag = 1.0/mag();
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

			T absVal = (T)fabs(m_dCoords[i]);

			if(absVal > max)
			{
				max = absVal;
				iResult = i;
			}//end if

		}//end for

		return iResult;

	}//end FindMaxAbsComponent
	
	template <typename Templateparm>
	friend std::ostream& operator<<(std::ostream& out, const CVector4<Templateparm>& v1); 
	
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
template<typename T> CVector4<T> operator*(T num, const CVector4<T> &vRHS);

template<typename T>
CVector4<T> operator*(T num, const CVector4<T> &vRHS) 
{
	// Return scaled vector
	return CVector4<T>(vRHS.x * num, vRHS.y * num, vRHS.z * num);
}//end  operator

/* typedefs to create float and double vectors */
typedef CVector4<double> CVector4d;
typedef CVector4<float>  CVector4f;

}

#endif  //_CVector4_H_
