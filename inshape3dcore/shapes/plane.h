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

#ifndef PLANE_H
#define PLANE_H

//===================================================
//					DEFINITIONS
//===================================================


//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include <vector>
#include <limits>
#include <vector3.h>
#include <shape.h>

namespace i3d {

/** @brief A plane in 3d space
 *
 * A plane in 3d space
 */  
template <typename T>
class Plane : public Shape<T>
{
public:
	Plane(void);
	Plane(const Vector3<T> &vOrig, const Vector3<T> &vNormal);
	~Plane(void);

/** 
 *
 * Returns the volume 
 * @return Returns zero volume
 */
	inline T getVolume() const {return (T)0.0;};

/**
 * Returns whether if query point is inside
 * @return Returns true when the query point is inside
 */
  bool isPointInside(const Vector3<T> &vQuery) const
  {
    return false; 
  }

  /**
   * Returns the geometric center of the shape
   *
   */
  Vector3<T> getCenter() const {return m_vOrigin;};

  /**
   * Returns an axis-aligned bounding box for the plane
   */
  AABB3<T> getAABB() {Vector3<T> vCenter = m_vOrigin + m_Extends[2] * m_vNormal;return AABB3<T>(vCenter,m_Extends);};

	Vector3<T> m_vNormal;

	Vector3<T> m_vOrigin;

	Vector3<T> m_vU;

	Vector3<T> m_vV;

  T m_dU;
  T m_dV;
  T m_dW;

  T m_Extends[3];

};

typedef Plane<float> Planef;
typedef Plane<double> Planed;
typedef Plane<Real> Planer;

}

#endif
