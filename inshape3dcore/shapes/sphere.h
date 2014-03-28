/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

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

#ifndef SPHERE_H_
#define SPHERE_H_

//===================================================
//					INCLUDES
//===================================================
#include <limits>
#include <vector3.h>
#include <shape.h>
#include <convexshape.h>

namespace i3d {

/** @brief A class for a sphere
 *
 * A class for a sphere with various sphere-related operations
 */      
template<class T>
class CSphere : public ConvexShape<T>
{
public:
	CSphere(void);

	CSphere(const CVector3<T> &vCenter, T dRad): m_vCenter(vCenter), m_Rad(dRad)
	{
	}//end

	~CSphere(void);

	CSphere(const CSphere<T> &copy)
	{
		this->m_Rad = copy.m_Rad;
		this->m_vCenter = copy.m_vCenter;
	}

	inline CVector3<T> eval(T phi, T theta) const
	{
	  return CVector3<T>(m_vCenter.x+m_Rad*cos(phi)*cos(theta), 
						 m_vCenter.y+m_Rad*cos(phi)*sin(theta), 
						 m_vCenter.z+m_Rad*sin(phi));
	}

/** 
 *
 * Returns the radius of the sphere
 * @return Returns the radius
 */
	inline T Radius() {return m_Rad;};

/** 
 *
 * Returns the radius of the sphere const version
 * @return Returns the radius
 */
	inline T Radius() const {return m_Rad;};

/**
 *
 * Returns the center of the sphere
 * @return Returns the center of the sphere
 */
	inline CVector3<T>& Center() {return m_vCenter;};

/**
 *
 * Returns the center of the sphere
 * @return Returns the center of the sphere
 */
	inline const CVector3<T>& Center() const {return m_vCenter;};

/** 
 *
 * Returns the volume of the sphere
 * @return Returns the volume of the sphere
 */
	inline T Volume() const {return (T)4.0/(T)3.0*3.14*m_Rad*m_Rad*m_Rad;};

/**
 * Returns a bounding box for the sphere
 * @return The axis-aligned bounding box of the sphere
 */
	CAABB3<T> GetAABB();

  CVector3<T> GetSupport(const CVector3<T> &v) const
  {
    if(v.mag() < CMath<T>::TOLERANCEZERO)
      return CVector3<T>(m_Rad,0,0);
    else
      return m_vCenter + m_Rad * (v*(1.0/v.mag()));
  };

  CVector3<T> GetPointOnBoundary() const
  {
    return m_vCenter + CVector3<T>(m_Rad,0,0);
  };

/**
 * Returns whether if query point is inside
 * @return Returns true when the query point is inside
 */
  bool PointInside(const CVector3<T> &vQuery) const
  {
    CVector3<T> diff = (vQuery - m_vCenter);
    return (diff * diff <= m_Rad * m_Rad); 
  }
  
  /**
   * Returns the geometric center of the shape
   *
   */
  CVector3<T> GetCenter() const {return m_vCenter;};

	CVector3<T> m_vCenter;
	T           m_Rad;

};



typedef CSphere<float> CSpheref;
typedef CSphere<double> CSphered;
typedef CSphere<Real> CSpherer;

}

#endif
