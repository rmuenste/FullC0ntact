/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#ifndef DISTANCETRIANGLE_H
#define DISTANCETRIANGLE_H
#include <vector3.h>
#include <mymath.h>
#include <triangle3.h>
#include <distance.h>

namespace i3d {

/**
* @brief Computes the distance between a point and a triangle in 3d
*
* Computes the distance between a point and a triangle in 3d
*
*/
template <class T>
class CDistancePointTriangle : public CDistance<T>
{
	public:  
  CDistancePointTriangle(const Triangle3<T> &face,const Vector3<T> &vQuery);
  ~CDistancePointTriangle();
  
  T ComputeDistance();
  T ComputeDistanceSqr();
  
  Vector3<T> m_vQuery;
  const Triangle3<T> *m_pTriangle;

  T ds,dt;

	/**
  * This is the query point
  */
	using CDistance<T>::m_vClosestPoint0;

	/**
  * This is closest point on the triangle
  */
	using CDistance<T>::m_vClosestPoint1;

  
};

}

#endif // DISTANCETRIANGLE_H
