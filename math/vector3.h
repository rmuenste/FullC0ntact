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
#if !defined(_Vector3_H)
#define _Vector3_H

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
class Vector3 {

public:

	/* constructor */
  host_dev
	Vector3(T a, T b, T c): x(a), y(b), z(c) {}

	/* copy constructor */
  host_dev
	Vector3(const Vector3 &v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
	}

    /* default constructor */
  host_dev
	Vector3():x(0), y(0), z(0){}

  host_dev
	~Vector3(){};

  host_dev
	inline const Vector3& operator=(const Vector3& v)
	{
		
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}//end  operator

  host_dev
	inline Vector3 operator - () const
	{
		return Vector3(-x,-y,-z);
	}//end operator

  host_dev
	inline const Vector3& operator=(const Vector4<T>& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}//end  operator

  host_dev
	inline Vector3 operator+(Vector3 v) const
	{
		return Vector3(x + v.x, y + v.y, z + v.z);
	}//end  operator

  host_dev
	inline Vector3 operator-(Vector3 v) const
	{
		return Vector3(x - v.x, y - v.y, z - v.z);
	}//end  operator

  host_dev
	inline Vector3 operator*(T num) const
	{
		// Return scaled vector
		return Vector3(x * num, y * num, z * num);
	}//end  operator

  host_dev
	inline Vector3 operator /(T &rhs)
	{
		return Vector3(x/rhs, y/rhs, z/rhs);
	}//end  operator

  host_dev
	inline T operator * (const Vector3 &rhs) const
	{
		return  x * rhs.x +
				y * rhs.y +
				z * rhs.z;
				
	}//end  operator

  host_dev
	inline T norm2()
	{
		return  (x * x) + (y * y) + (z * z);
	}//end  operator

  host_dev
	inline T norm2() const
	{
		return  (x * x) + (y * y) + (z * z);
	}//end  operator

  host_dev
	inline T mag()
	{
		return std::sqrt(norm2());
	}//end  operator

  host_dev
	inline T mag() const
	{
		return std::sqrt(norm2());
	}//end  operator

	inline void Normalize()
	{
	  T magnitude = mag();
      T dInvMag = 1.0/magnitude;
	  if(std::isinf(dInvMag))
	    return;

		x *= (T)dInvMag;
		y *= (T)dInvMag;
		z *= (T)dInvMag;
	}//end Normalize

  host_dev
  inline void normalize()
  {
    T magnitude = mag();
    T dInvMag = 1.0 / magnitude;

    x *= (T)dInvMag;
    y *= (T)dInvMag;
    z *= (T)dInvMag;
  }//end Normalize

  host_dev
	inline static Vector3 createVector(const Vector3 &a, const Vector3 &b)
	{
		Vector3 res = b - a;
		return res;
	}//end  operator

  host_dev
	inline const Vector3& operator /= (const T &rhs)
	{
		x /= rhs;
		y /= rhs;
		z /= rhs;
		return *this;
	}//end  operator

  host_dev
	inline const Vector3& operator += (const Vector3 &rhs)
	{

		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		return *this;
	}//end  operator

  host_dev	
	inline const Vector3& operator -= (const Vector3 &rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		return *this;
	}//end  operator

  host_dev
	inline const Vector3& operator *= (const T &d)
	{
		x *= d;
		y *= d;
		z *= d;
		return *this;
	}//end  operator

  inline Vector3 largestComponentDir() const
  {
    if (std::abs(x) > std::abs(y))
    {
      if (std::abs(x) > std::abs(z))
      {
        if (x > 0.0)
        {
          Vector3 n(1.0, 0.02, 0.02);
          n.Normalize();
          return n;
        }
        else
        {
          Vector3 n(-1.0, 0.02, 0.02);
          n.Normalize();
          return n;
        }
      }
      else
      {
        if (z > 0.0)
        {
          Vector3 n(0.02, 0.02, 1);
          n.Normalize();
          return n;
        }
        else
        {
          Vector3 n(0.02, 0.02, -1);
          n.Normalize();
          return n;
        }
      }
    }
    else
    {
      if (std::abs(y) > std::abs(z))
      {
        if (y > 0.0)
        {
          Vector3 n(0.02, 1, 0.03);
          n.Normalize();
          return n;
        }
        else
        {
          Vector3 n(0.02, -1, 0.03);
          n.Normalize();
          return n;
        }
      }
      else
      {
        if (z > 0.0)
        {
          Vector3 n(0.01, 0.02, 1);
          n.Normalize();
          return n;
        }
        else
        {
          Vector3 n(0.01, 0.02, -1);
          n.Normalize();
          return n;
        }
      }
    }
  }

  inline static T dot(const Vector3 &a, const Vector3 &b)
  {
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
  }//end  operator

  inline static T AngleBetween(const Vector3 &a, const Vector3 &b)
  {
    T lengthA = (T)a.mag();
    T lengthB = (T)b.mag();

    T cosAngle = dot(a,b)/(lengthA*lengthB);

    return acos(cosAngle);

  }//end AngleBetween


  static void GenerateComplementBasis (Vector3& u, Vector3& v, const Vector3& w);

  host_dev
    inline static Vector3 Cross(Vector3 vVector1, Vector3 vVector2)
    {
      Vector3 vCross;

      vCross.x = ((vVector1.y * vVector2.z) - (vVector1.z * vVector2.y));

      vCross.y = ((vVector1.z * vVector2.x) - (vVector1.x * vVector2.z));

      vCross.z = ((vVector1.x * vVector2.y) - (vVector1.y * vVector2.x));

      return vCross;
    }
  template<typename Templateparm>
    friend std::ostream& operator<< (std::ostream& out, const Vector3<Templateparm>& v1);

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

template<class T>
host_dev
Vector3<T> operator*(T a,const Vector3<T> &vRHS);


template<class T>
host_dev
Vector3<T> operator*(T a,const Vector3<T> &vRHS)
{
  // Return scaled vector
  return Vector3<T>(vRHS.x * a, vRHS.y * a,vRHS.z * a);
}//end  operator

  template<class T>
void Vector3<T>::GenerateComplementBasis (Vector3<T> &u, Vector3<T> &v, const Vector3<T> &w)
{
  Real invLength;

  if ( std::abs(w.m_dCoords[0]) >= std::abs(w.m_dCoords[1]) )
  {
    // W.x or W.z is the largest magnitude component, swap them
    invLength = T(1.0)/std::sqrt(w.m_dCoords[0]*w.m_dCoords[0] + w.m_dCoords[2]*w.m_dCoords[2]);
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
    invLength = T(1.0)/std::sqrt(w.m_dCoords[1]*w.m_dCoords[1] + w.m_dCoords[2]*w.m_dCoords[2]);
    u.m_dCoords[0] = (T)0;
    u.m_dCoords[1] = +w.m_dCoords[2]*invLength;
    u.m_dCoords[2] = -w.m_dCoords[1]*invLength;
    v.m_dCoords[0] = w.m_dCoords[1]*u.m_dCoords[2] - w.m_dCoords[2]*u.m_dCoords[1];
    v.m_dCoords[1] = -w.m_dCoords[0]*u.m_dCoords[2];
    v.m_dCoords[2] = w.m_dCoords[0]*u.m_dCoords[1];
  }
}

template<class T> std::ostream& operator<<(std::ostream& out, const Vector3<T> &v1)
{
  return out << "["<<v1.x<<","<<v1.y<<","<<v1.z<<"]"<<std::endl;
}

/* typedefs to create double and double vectors */
typedef Vector3<double> Vector3d;
typedef Vector3<double>  Vector3f;
typedef Vector3<Real> VECTOR3;
typedef Vector3<Real> v3d;
typedef Vector3<Real> Vec3;
typedef Vector3<float> vector3;

}

#endif  //_Vector3_H
