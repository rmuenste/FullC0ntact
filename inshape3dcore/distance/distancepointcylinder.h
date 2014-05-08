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

#ifndef DISTANCEPOINTREC_H
#define DISTANCEPOINTREC_H

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
#include <cylinder.h>
#include <line3.h>
#include <distance.h>

namespace i3d {

/**
* @brief Computes the distance between a point and a cylinder
*
* Computes the distance between a point and a cylinder
*
*/   
template <typename T>
class CDistancePointCylinder : public CDistance<T>
{
public:
  CDistancePointCylinder(void);
  ~CDistancePointCylinder(void);
  CDistancePointCylinder (const Vector3<T>& point, const Cylinder<T>& cylinder);

  T ComputeDistanceSqr();
  T ComputeDistance();

  Cylinder<T> m_Cylinder;
  Vector3<T> m_vPoint;

  /**
  * The input point is the closest point
  */
  using CDistance<T>::m_vClosestPoint0;

  /**
  * The closest point on the rectangle
  */  
  using CDistance<T>::m_vClosestPoint1;

};

}

#endif
