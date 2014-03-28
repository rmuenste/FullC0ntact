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


#include "distanceaabbaabb.h"

namespace i3d {

template <typename T>
CDistanceAabbAabb<T>::CDistanceAabbAabb(const CAABB3<T> &rAABB1, const CAABB3<T> &rAABB2)  : m_pAABB1(&rAABB1), m_pAABB2(&rAABB2)
{

}

template <typename T>
CDistanceAabbAabb<T>::~CDistanceAabbAabb() 
{

}

template <typename T>
T CDistanceAabbAabb<T>::ComputeDistanceSqr()
{
  
  CVector3<T> vD(0,0,0);  

  for(int j=0;j<3;j++)
  {
    if(m_pAABB1->vertices_[0].m_dCoords[j] > m_pAABB2->vertices_[1].m_dCoords[j])
    {
      vD.m_dCoords[j]=m_pAABB1->vertices_[0].m_dCoords[j] - m_pAABB2->vertices_[1].m_dCoords[j];
    }
    else if(m_pAABB2->vertices_[0].m_dCoords[j] > m_pAABB1->vertices_[1].m_dCoords[j])
    {
      vD.m_dCoords[j]=m_pAABB2->vertices_[0].m_dCoords[j] - m_pAABB1->vertices_[1].m_dCoords[j];
    }
    else
      vD.m_dCoords[j]=T(0.0);
  }
  
  return vD.x*vD.x+vD.y*vD.y+vD.z*vD.z;
  
}

template <typename T>
T CDistanceAabbAabb<T>::ComputeDistance()
{
  return sqrt(ComputeDistanceSqr());
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceAabbAabb<float>;

template class CDistanceAabbAabb<double>;

}