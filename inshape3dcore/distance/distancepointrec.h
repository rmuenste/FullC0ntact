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
#include <rectangle3.h>
#include <line3.h>
#include <distance.h>
#include <plane.h>

namespace i3d {

/**
* @brief Computes the distance between a point and a rectangle in 3d
*
* Computes the distance between a point and a rectangle in 3d
*
*/   
template <typename T>
class CDistancePointRec : public CDistance<T>
{
public:
  CDistancePointRec(void);
  ~CDistancePointRec(void);
  CDistancePointRec (const Vector3<T>& point, const Rectangle3<T>& rectangle);

  T ComputeDistanceSqr();
  T ComputeDistance();

  Rectangle3<T> m_Rec;
  Vector3<T> m_vPoint;

  T m_ParamRectangle[2];

  /**
  * The input point is the closest point
  */
  using CDistance<T>::m_vClosestPoint0;

  /**
  * The closest point on the rectangle
  */  
  using CDistance<T>::m_vClosestPoint1;

};

template <typename T>
class DistancePointPlane : public CDistance<T>
{
public:
  DistancePointPlane(void);
  ~DistancePointPlane(void);
  DistancePointPlane (const Vector3<T>& point, const Plane<T>& plane);

  T ComputeDistanceSqr();
  T ComputeDistance();

  Plane<T> plane_;
  Vector3<T> m_vPoint;

  T m_ParamRectangle[2];

  /**
  * The input point is the closest point
  */
  using CDistance<T>::m_vClosestPoint0;

  /**
  * The closest point on the plane
  */  
  using CDistance<T>::m_vClosestPoint1;

};


}

#endif
