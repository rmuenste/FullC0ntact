/***************************************************************************
 *   Copyright (C) 2006 by Raphael Münster   *
 *   raphael@Cortez   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/




#if !defined(_CVector3_H)
#define _CVector3_H

//===================================================
//					INCLUDES
//===================================================

#include <iostream>
#include "mymath.h"
#include "vector4.h"


namespace i3d {

/**
* @brief A class for a 3d vector and various 3d vector operations
*
* A class for a 3d vector and various 3d vector operations
*/    
template<class T>
class CVector3 {

public:
	/* constructor */
	CVector3(T a, T b, T c): x(a), y(b), z(c) {}

	/* copy constructor */
	CVector3(const CVector3 &v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
	}

    /* default constructor */
	CVector3():x(0), y(0), z(0){}
	~CVector3(){};
	
	inline const CVector3& operator=(const CVector3& v)
	{
		
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}//end  operator

	inline CVector3 operator - () const
	{
		return CVector3(-x,-y,-z);
	}//end operator

	inline const CVector3& operator=(const CVector4<T>& v)
	{
		
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}//end  operator

	inline CVector3 operator+(CVector3 v) const
	{
		return CVector3(x + v.x, y + v.y, z + v.z);
	}//end  operator

	inline CVector3 operator-(CVector3 v) const
	{
		return CVector3(x - v.x, y - v.y, z - v.z);
	}//end  operator

	inline CVector3 operator*(T num) const
	{
		// Return scaled vector
		return CVector3(x * num, y * num, z * num);
	}//end  operator



	inline T operator * (const CVector3 &rhs) const
	{
		return  x * rhs.x +
				y * rhs.y +
				z * rhs.z;
				
	}//end  operator

	inline T norm2()
	{
		return  (x * x) + (y * y) + (z * z);
	}//end  operator

	inline T norm2() const
	{
		return  (x * x) + (y * y) + (z * z);
	}//end  operator


	inline double mag()
	{
		return sqrt(norm2());
	}//end  operator

	inline double mag() const
	{
		return sqrt(norm2());
	}//end  operator


	inline void Normalize()
	{
		double dInvMag = 1.0/mag();
		x *= (T)dInvMag;
		y *= (T)dInvMag;
		z *= (T)dInvMag;
	}//end Normalize

	inline static CVector3 createVector(const CVector3 &a, const CVector3 &b)
	{
		CVector3 res = b - a;
		return res;
	}//end  operator


	inline const CVector3& operator /= (const T &rhs)
	{
		x /= rhs;
		y /= rhs;
		z /= rhs;
		return *this;
	}//end  operator


	inline const CVector3& operator += (const CVector3 &rhs)
	{

		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		return *this;
	}//end  operator

	
	inline const CVector3& operator -= (const CVector3 &rhs)
	{

		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		
		return *this;
	}//end  operator

	
	inline const CVector3& operator *= (const T &d)
	{
		x *= d;
		y *= d;
		z *= d;
		
		return *this;
	}//end  operator

	inline static T dot(const CVector3 &a, const CVector3 &b)
	{
		return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
	}//end  operator

	inline static T AngleBetween(const CVector3 &a, const CVector3 &b)
	{
		T lengthA = (T)a.mag();
		T lengthB = (T)b.mag();

		T cosAngle = dot(a,b)/(lengthA*lengthB);

		return acos(cosAngle);

	}//end AngleBetween


	static void GenerateComplementBasis (CVector3& u, CVector3& v, const CVector3& w);
		
	inline static CVector3 Cross(CVector3 vVector1, CVector3 vVector2)
	{
		CVector3 vCross;

		vCross.x = ((vVector1.y * vVector2.z) - (vVector1.z * vVector2.y));

		vCross.y = ((vVector1.z * vVector2.x) - (vVector1.x * vVector2.z));

		vCross.z = ((vVector1.x * vVector2.y) - (vVector1.y * vVector2.x));

		return vCross;
	}
	template<typename Templateparm>
	friend std::ostream& operator<< (std::ostream& out, const CVector3<Templateparm>& v1); 

	/* union to allow different access methods */
	union
	{
		T m_dCoords[3];
		struct
		{
			T x;
			T y;
			T z;
		};
	};
	
		
};

template<class T> CVector3<T> operator*(T a,const CVector3<T> &vRHS);


template<class T> CVector3<T> operator*(T a,const CVector3<T> &vRHS)
{
	// Return scaled vector
	return CVector3<T>(vRHS.x * a, vRHS.y * a,vRHS.z * a);
}//end  operator

template<class T>
void CVector3<T>::GenerateComplementBasis (CVector3<T> &u, CVector3<T> &v, const CVector3<T> &w)
{
    Real invLength;

    if ( fabs(w.m_dCoords[0]) >= fabs(w.m_dCoords[1]) )
    {
        // W.x or W.z is the largest magnitude component, swap them
        invLength = 1.0/sqrt(w.m_dCoords[0]*w.m_dCoords[0] + w.m_dCoords[2]*w.m_dCoords[2]);
        u.m_dCoords[0] = -w.m_dCoords[2]*invLength;
        u.m_dCoords[1] = (T)0;
        u.m_dCoords[2] = +w.m_dCoords[0]*invLength;
        v.m_dCoords[0] = w.m_dCoords[1]*u.m_dCoords[2];
        v.m_dCoords[1] = w.m_dCoords[2]*u.m_dCoords[0] - w.m_dCoords[0]*u.m_dCoords[2];
        v.m_dCoords[2] = -w.m_dCoords[1]*u.m_dCoords[0];
    }
    else
    {
        // W.y or W.z is the largest magnitude component, swap them
        invLength = 1.0/sqrt(w.m_dCoords[1]*w.m_dCoords[1] + w.m_dCoords[2]*w.m_dCoords[2]);
        u.m_dCoords[0] = (T)0;
        u.m_dCoords[1] = +w.m_dCoords[2]*invLength;
        u.m_dCoords[2] = -w.m_dCoords[1]*invLength;
        v.m_dCoords[0] = w.m_dCoords[1]*u.m_dCoords[2] - w.m_dCoords[2]*u.m_dCoords[1];
        v.m_dCoords[1] = -w.m_dCoords[0]*u.m_dCoords[2];
        v.m_dCoords[2] = w.m_dCoords[0]*u.m_dCoords[1];
    }
}

template<class T> std::ostream& operator<<(std::ostream& out, const CVector3<T> &v1) 
{
	return out << "("<<v1.x<<","<<v1.y<<","<<v1.z<<")"<<std::endl;
}

/* typedefs to create double and double vectors */
typedef CVector3<double> CVector3d;
typedef CVector3<double>  CVector3f;
typedef CVector3<Real> VECTOR3;

}

#endif  //_CVector3_H
