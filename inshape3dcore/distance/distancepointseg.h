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

#ifndef DISTANCEPOINTSEG_H
#define DISTANCEPOINTSEG_H

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
#include <segment3.h>
#include <line3.h>
#include <distance.h>

 
namespace i3d {

/**
* @brief Computes the distance between a point and a line segment in 3d
*
* Computes the distance between a point and a line segment in 3d
*
*/  
template <typename T>
class CDistancePointSeg : public CDistance<T>
{
public:
  
  CDistancePointSeg(void);
  CDistancePointSeg (const Vector3<T>& point, const Segment3<T>& segment);
  ~CDistancePointSeg(void);

  T ComputeDistanceSqr();
  T ComputeDistance();

  Segment3<T> m_Seg;
  Vector3<T> m_vPoint;

  T m_ParamSegment;

  //this is m_vPoint
  using CDistance<T>::m_vClosestPoint0;
  //the closest Point on the segment
  using CDistance<T>::m_vClosestPoint1;

};

}

#endif
