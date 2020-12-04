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


//===================================================
//                     INCLUDES
//===================================================


#include "distanceaabbpoint.h"

namespace i3d {

template <typename T>

CDistanceAabbPoint<T>::CDistanceAabbPoint(const AABB3<T> &rAABB1, const Vector3<T> &vector)  : m_pAABB1(&rAABB1), point_(&vector)
{

}

template <typename T>
CDistanceAabbPoint<T>::~CDistanceAabbPoint() 
{

}

template <typename T>
T CDistanceAabbPoint<T>::ComputeDistanceSqr()
{
    const Vector3<T>& p = *point_;
    const AABB3<T>& b = *m_pAABB1;
    T sqDist = 0.0;
    for (int i = 0; i < 3; i++) {
      T v = p.m_dCoords[i];
      if (v < b.vertices_[0].m_dCoords[i]) sqDist += (b.vertices_[0].m_dCoords[i] - v) * (b.vertices_[0].m_dCoords[i] - v);
      if (v > b.vertices_[1].m_dCoords[i]) sqDist += (v - b.vertices_[1].m_dCoords[i]) * (v - b.vertices_[1].m_dCoords[i]);
    }

    return sqDist;
}

template <typename T>
T CDistanceAabbPoint<T>::ComputeDistance()
{
  return sqrt(ComputeDistanceSqr());
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceAabbPoint<float>;

template class CDistanceAabbPoint<double>;

}