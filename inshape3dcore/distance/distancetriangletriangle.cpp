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
#include "distancetriangletriangle.h"
#include <distancesegseg.h>
#include <distancetriangle.h>


namespace i3d {

template <typename T>
CDistanceTriangleTriangle<T>::CDistanceTriangleTriangle() 
{

}

template <typename T>
CDistanceTriangleTriangle<T>::~CDistanceTriangleTriangle() 
{

}

template <typename T>
T CDistanceTriangleTriangle<T>::ComputeDistance()
{
	return sqrt(ComputeDistanceSqr());
}

template <typename T>
T CDistanceTriangleTriangle<T>::ComputeDistanceSqr()
{

  T sqrDist=std::numeric_limits<T>::max();

  //distance between edges
  int i0,i1;
  //edges of the first triangle
  for(i0=2,i1=0; i1<3; i0=i1++)
  {
    int j0,j1;
    //edges of the second triangle
    for(j0=2,j1=0; j1<3; j0=j1++)
    {
      Segment3<T> seg0(m_Tri0.Get(i0),m_Tri0.Get(i1));
      Segment3<T> seg1(m_Tri1.Get(j0),m_Tri1.Get(j1));
      CDistanceSegSeg<T> distanceSegSeg(seg0,seg1);
      T dist = distanceSegSeg.ComputeDistanceSqr();
      if(sqrDist > dist)
      {
        sqrDist = dist;
        m_vClosestPoint0 = distanceSegSeg.m_vClosestPoint0;
        m_vClosestPoint1 = distanceSegSeg.m_vClosestPoint1;
      }
    }
  }

  //compute distance between point on triangle1 and triangle0
  for(i0=0; i0<3; i0++)
  {
    CDistancePointTriangle<T> distPointTriangle(m_Tri1,m_Tri0.Get(i0));
    T dist = distPointTriangle.ComputeDistanceSqr();
    if(sqrDist > dist)
    {
      sqrDist = dist;
      m_vClosestPoint0 = distPointTriangle.m_vClosestPoint0;
      m_vClosestPoint1 = distPointTriangle.m_vClosestPoint1;
    }

  }//end for

  //compute distance between point on triangle0 and triangle1
  for(i0=0; i0<3; i0++)
  {
    CDistancePointTriangle<T> distPointTriangle(m_Tri0,m_Tri1.Get(i0));
    T dist = distPointTriangle.ComputeDistanceSqr();
    if(sqrDist > dist)
    {
      sqrDist = dist;
      m_vClosestPoint0 = distPointTriangle.m_vClosestPoint0;
      m_vClosestPoint1 = distPointTriangle.m_vClosestPoint1;
    }

  }//end for

	return sqrDist;

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceTriangleTriangle<Real>;
}
