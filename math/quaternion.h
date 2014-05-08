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

//===================================================
//			DEFINES
//===================================================

#if !defined _QUATERNION_
#define _QUATERNION_

#define PI 3.14159

//===================================================
//           		INCLUDES
//===================================================
#include <iostream>
#include <math.h>
#include "matrix4x4.h"
#include "vector3.h"

namespace i3d {

/**
* @brief A class for a quaternion
*
* A class for a quaternion
*/  
template<class T>
class CQuaternion
{

public:

	/* default constructor */
	CQuaternion() : x(0), y(0), z(0), w(1) {};

	CQuaternion(T a, T b, T c, T d) : x(a), y(b), z(c), w(d) {};

	/* copy constructor */
	CQuaternion( const CQuaternion &q ) : x(q.x), y(q.y), z(q.z), w(q.w)
	{

	};

	inline void SetValues(T X, T Y, T Z, T W)
	{
		x = X;
		y = Y;
		z = Z;
		w = W;
	}//end SetValues

  inline CQuaternion operator+(CQuaternion v) const
  {
    return CQuaternion(x + v.x, y + v.y, z + v.z, w + v.w);
  };//end  operator

  inline CQuaternion operator-(CQuaternion v) const
  {
    return CQuaternion(x - v.x, y - v.y, z - v.z, w - v.w);
  };//end  operator

	inline CQuaternion operator - () const
	{
		return CQuaternion(-x,-y,-z,w);
	};

	inline T norm2()
	{
		return  (x * x) + (y * y) + (z * z) + (w * w);
	}//end  operator

	inline double Mag()
	{
		return sqrt(norm2());
	}//end  Mag

	inline void Normalize()
	{
		double InvMag = 1.0/Mag();
		x*= (T)InvMag;
		y*= (T)InvMag;
		z*= (T)InvMag;
		w*= (T)InvMag;
	}//end Normalize

	inline const CQuaternion& GetNormalized()
	{
		double InvMag = 1.0/Mag();
		x*= (T)InvMag;
		y*= (T)InvMag;
		z*= (T)InvMag;
		w*= (T)InvMag;
		return *this;
	}//end GetNormalized

  inline CQuaternion operator *(T s) const
  {
    return CQuaternion(x*s,y*s,z*s,w*s);
  }

	inline CQuaternion operator *(const CQuaternion &q) const
	{
		Vector3<T> v1(x,y,z);
		Vector3<T> v2(q.x, q.y, q.z);

		//Quaternion multiplication:
    //Scalar component = w1*w2 - vec1*vec2
    //Vector component = vec2*w1 + vec1*w2 + Cross(vec1,vec2)
		T nS = w*q.w - (v1 * v2);

		Vector3<T> q_vec = v2 * w + v1 * q.w + (Vector3<T>::Cross(v1,v2));

		CQuaternion result(q_vec.x, q_vec.y, q_vec.z, nS);

		result.Normalize();

		return result;

	}//end operator *

	inline static CQuaternion GetQuaternion(const Vector3<T> &v1, const Vector3<T> &v2)
	{

		CQuaternion q;

		T x = v1.y * v2.z - v1.z * v2.y;

		T y = v1.z * v2.x - v1.x * v2.z;

		T z = v1.x * v2.y - v1.y * v2.x;

		T w = (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);

		q.SetValues(x,y,z,w);

		return q;

	}//end constructor
	
/**
* Converts the quaternion to a rotation matrix
*
* @param pMatrix the rotation matrix
*/
	void CreateMatrix(CMatrix4x4<T> &pMatrix);

/**
* Converts the quaternion to a rotation matrix
*
* @param pMatrix the rotation matrix
*/  
	void CreateMatrix(CMatrix4x4<T> &pMatrix) const;

/**
* Returns a rotation matrix correspoinding to the quaternion
* 
* @Return Returns a transformation matrix
*/  
  CMatrix3x3<T> GetMatrix() const;
  
/**
* Creates the quaternion from euler angles (heading=y,attitude=z,bank=x)
*/  
  void CreateFromEulerAngles(T y, T z, T x);  
  
  
/**
* Constructs a quaternion from an axis angle rotation
*
* @param X the x-coordinate of the axis
* @param Y the y-coordinate of the axis
* @param Z the z-coordinate of the axis
* @param W the rotation around the axis
*/  
	void AxisAngleToQuat(T X, T Y, T Z, T W);

/**
* Constructs a quaternion from an axis angle rotation
*
* @param vAxis the rotation axis
* @param W the rotation around the axis
*/    
	void AxisAngleToQuat(Vector3<T> vAxis, T W);

	union
	{
		T m_Data[4];

		struct
		{
			T x;
			T y;
			T z;
			T w;
		};
	};

};//end class Quaternion

template<class T> CQuaternion<T> operator*(T a,const CQuaternion<T> &vRHS);


template<class T> CQuaternion<T> operator*(T a,const CQuaternion<T> &vRHS)
{
  // Return scaled vector
  return CQuaternion<T>(vRHS.x * a, vRHS.y * a,vRHS.z * a, vRHS.w * a);
}//end  operator

typedef CQuaternion<float> CQuaternionf;
typedef CQuaternion<double> CQuaterniond;
typedef CQuaternion<Real> CQuaternionr;

}

#endif
