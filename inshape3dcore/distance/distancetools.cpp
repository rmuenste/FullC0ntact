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


#include "distancetools.h"

namespace i3d {

template <class T>
CDistanceTools<T>::CDistanceTools() 
{

}

template <class T>
CDistanceTools<T>::~CDistanceTools() 
{

}

template <class T>
unsigned int CDistanceTools<T>::ClassifyVertex(const CVector3<T> &vertex, const COBB3<T> &box)
{

  //classify every vertex
    unsigned int iRegion=0;
    if(vertex.x < -box.m_Extents[0])
    {
      iRegion |= 0x01;
    }
    if(vertex.x >= box.m_Extents[0])
    {
      iRegion |= 0x02;
    }
    if(vertex.y < -box.m_Extents[1])
    {
      iRegion |= 0x04;
    }
    if(vertex.y >= box.m_Extents[1])
    {
      iRegion |= 0x08;
    }
    if(vertex.z < -box.m_Extents[2])
    {
      iRegion |= 0x10;
    }
    if(vertex.z >= box.m_Extents[2])
    {
      iRegion |= 0x20;
    }
    return iRegion;
}

template <typename T>
CVector3<T> CDistanceTools<T>::GetRegionVertex(unsigned int iRegion, const COBB3<T> &box)
{
  CVector3<T> vVertex;
  for(unsigned int i=1;i<=3;i++)
  {
    unsigned int m = 1;
    //left shift
    m <<= 2*(i-1);
    if((iRegion & m) != 0)
      vVertex.m_dCoords[i-1]=-box.m_Extents[i-1];
    else
      vVertex.m_dCoords[i-1]=box.m_Extents[i-1];
  }
  return vVertex;
}

template <typename T>
CSegment3<T> CDistanceTools<T>::GetRegionEdge(unsigned int iRegion, const COBB3<T> &box)
{
  //the vertex region of the first vertex of the edge
  unsigned int c1;
  //the vertex region of the 2nd vertex of the edge
  unsigned int c2;
  int j=0;

  unsigned int m1 = 1;

  CVector3<T> vDir;
  //identify the double zero pattern 00****,**00**,****00
  for(unsigned int i=1;i<=3;i++)
  {
    unsigned int m3 = 3;
    m3 <<= 2*(i-1);
    if((iRegion & m3) == 0)
    {
      vDir.m_dCoords[i-1] = 1;
      j=i;
    }
    else
      vDir.m_dCoords[i-1] = 0;
  }

  m1 = 1;
  m1 <<=2*(j-1);
  c1 = iRegion ^ m1;
  m1 = 1;
  m1 <<=2*(j-1)+1;
  c2 = iRegion ^ m1;

  //get the vertex corresponding to code c1 of Box iwhich
  CVector3<T> vA = GetRegionVertex(c1,box);
  //get the vertex corresponding to code c2 of Box iwhich
  CVector3<T> vB = GetRegionVertex(c2,box);
  CSegment3<T> seg(vA,vB);
  return seg;
}

template <typename T>
CRectangle3<T> CDistanceTools<T>::GetRegionFace(unsigned int iRegion, const COBB3<T> &box)
{

  CRectangle3<T> rec;
  CVector3<T> vAxes[3] = {CVector3<T>(1,0,0),CVector3<T>(0,1,0),CVector3<T>(0,0,1)};
  CVector3<T> extAxis0 = box.m_Extents[0] * vAxes[0];
  CVector3<T> extAxis1 = box.m_Extents[1] * vAxes[1];
  CVector3<T> extAxis2 = box.m_Extents[2] * vAxes[2];

  switch(iRegion)
  {
    case 1:
      rec.m_vCenter= - extAxis0;
      rec.m_vUV[0]=vAxes[1];
      rec.m_vUV[1]=vAxes[2];
      rec.m_Extents[0]=box.m_Extents[1];
      rec.m_Extents[1]=box.m_Extents[2];
      break;
    case 2:
      rec.m_vCenter= extAxis0;
      rec.m_vUV[0]=vAxes[1];
      rec.m_vUV[1]=vAxes[2];
      rec.m_Extents[0]=box.m_Extents[1];
      rec.m_Extents[1]=box.m_Extents[2];
      break;
    case 4:
      rec.m_vCenter= - extAxis1;
      rec.m_vUV[0]=vAxes[0];
      rec.m_vUV[1]=vAxes[2];
      rec.m_Extents[0]=box.m_Extents[0];
      rec.m_Extents[1]=box.m_Extents[2];
      break;
    case 8:
      rec.m_vCenter= extAxis1;
      rec.m_vUV[0]=vAxes[0];
      rec.m_vUV[1]=vAxes[2];
      rec.m_Extents[0]=box.m_Extents[0];
      rec.m_Extents[1]=box.m_Extents[2];
      break;
    case 16:
      rec.m_vCenter= - extAxis2;
      rec.m_vUV[0]=vAxes[0];
      rec.m_vUV[1]=vAxes[1];
      rec.m_Extents[0]=box.m_Extents[0];
      rec.m_Extents[1]=box.m_Extents[1];
      break;
    case 32:
      rec.m_vCenter= extAxis2;
      rec.m_vUV[0]=vAxes[0];
      rec.m_vUV[1]=vAxes[1];
      rec.m_Extents[0]=box.m_Extents[0];
      rec.m_Extents[1]=box.m_Extents[1];
      break;
  }
  return rec;
}

template <typename T>
CVector3<T> CDistanceTools<T>::GetFaceNormal(unsigned int iRegion, const COBB3<T> &box)
{
  CVector3<T> vNormal;
  switch(iRegion)
  {
    case 1:
      vNormal =  -box.m_vUVW[0];
      break;
    case 2:
      vNormal =  box.m_vUVW[0];
      break;
    case 4:
      vNormal =  -box.m_vUVW[1];
      break;
    case 8:
      vNormal =  box.m_vUVW[1];
      break;
    case 16:
      vNormal =  -box.m_vUVW[2];
      break;
    case 32:
      vNormal =  box.m_vUVW[2];
      break;
  }
  return vNormal;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceTools<Real>;
//----------------------------------------------------------------------------

}