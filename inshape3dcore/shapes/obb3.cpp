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

#include "obb3.h"
#include <aabb3.h>
#include <stdlib.h>

namespace i3d {

template <typename T>
COBB3<T>::COBB3 ()
{
}

template <typename T>
COBB3<T>::~COBB3 ()
{
}


template <typename T>
COBB3<T>::COBB3 (const CVector3<T>& center, const CVector3<T> axis[3], const T extent[3])
{
		m_vCenter = center;
    m_vUVW[0] = axis[0];
    m_vUVW[1] = axis[1];
    m_vUVW[2] = axis[2];
    m_Extents[0] = extent[0];
    m_Extents[1] = extent[1];
    m_Extents[2] = extent[2];
}

template <typename T>
COBB3<T>::COBB3(const COBB3<T> &copy)
{
  m_vCenter = copy.m_vCenter;
  m_vUVW[0] = copy.m_vUVW[0];
  m_vUVW[1] = copy.m_vUVW[1];
  m_vUVW[2] = copy.m_vUVW[2];
  m_Extents[0] = copy.m_Extents[0];
  m_Extents[1] = copy.m_Extents[1];
  m_Extents[2] = copy.m_Extents[2];
}



template <typename T>
COBB3<T>::COBB3 (const CVector3<T>& center, const CVector3<T>& axis0,
    const CVector3<T>& axis1, const CVector3<T>& axis2,
    const T extent0, const T extent1, const T extent2)
    :
    m_vCenter(center)
{
    m_vUVW[0] = axis0;
    m_vUVW[1] = axis1;
    m_vUVW[2] = axis2;
    m_Extents[0] = extent0;
    m_Extents[1] = extent1;
    m_Extents[2] = extent2;
}

template <typename T>
void COBB3<T>::ComputeVertices (CVector3<T> vertex[8]) const
{
    CVector3<T> extAxis0 = m_Extents[0]*m_vUVW[0];
    CVector3<T> extAxis1 = m_Extents[1]*m_vUVW[1];
    CVector3<T> extAxis2 = m_Extents[2]*m_vUVW[2];

    vertex[0] = m_vCenter - extAxis0 - extAxis1 - extAxis2;
    vertex[1] = m_vCenter + extAxis0 - extAxis1 - extAxis2;
    vertex[2] = m_vCenter + extAxis0 + extAxis1 - extAxis2;
    vertex[3] = m_vCenter - extAxis0 + extAxis1 - extAxis2;
    vertex[4] = m_vCenter - extAxis0 - extAxis1 + extAxis2;
    vertex[5] = m_vCenter + extAxis0 - extAxis1 + extAxis2;
    vertex[6] = m_vCenter + extAxis0 + extAxis1 + extAxis2;
    vertex[7] = m_vCenter - extAxis0 + extAxis1 + extAxis2;
}

template <typename T>
bool COBB3<T>::PointInside (const CVector3<T> &vQuery) const
{

	CVector3<T> extAxis0 = m_Extents[0]*m_vUVW[0];
	CVector3<T> extAxis1 = m_Extents[1]*m_vUVW[1];
	CVector3<T> extAxis2 = m_Extents[2]*m_vUVW[2];

	CVector3<T> vTest;

	//point on top face
	CVector3<T> vMidTop = m_vCenter + extAxis2;
	//normal vector
	CVector3<T> vTop = m_vUVW[2];
	//compute the test vector
	vTest = vQuery - vMidTop;
	//check sign of scalar product
	if((vTop * vTest) > 0.0)
		return false;

	//point on bottom face
	CVector3<T> vMidBot = m_vCenter - extAxis2;
	//normal vector
	CVector3<T> vBot = -m_vUVW[2];
	//compute the test vector
	vTest = vQuery - vMidBot;
	//check sign of scalar product
	if((vBot * vTest) > 0.0)
		return false;

	//point on right face
	CVector3<T> vMidRight = m_vCenter + extAxis0;
	//normal vector
	CVector3<T> vRight = m_vUVW[0];
	//compute the test vector
	vTest = vQuery - vMidRight;
	//check sign of scalar product
	if((vRight * vTest) > 0.0)
		return false;

	//point on left face
	CVector3<T> vMidLeft = m_vCenter - extAxis0;
	//normal vector
	CVector3<T> vLeft = -m_vUVW[0];
	//compute the test vector
	vTest = vQuery - vMidLeft;
	//check sign of scalar product
	if((vLeft * vTest) > 0.0)
		return false;

	//point on back face
	CVector3<T> vMidBack = m_vCenter + extAxis1;
	//normal vector
	CVector3<T> vBack = m_vUVW[1];
	//compute the test vector
	vTest = vQuery - vMidBack;
	//check sign of scalar product
	if((vBack * vTest) > 0.0)
		return false;

	//point on front face
	CVector3<T> vMidFront = m_vCenter - extAxis1;
	//normal vector
	CVector3<T> vFront = -m_vUVW[1];
	//compute the test vector
	vTest = vQuery - vMidFront;
	//check sign of scalar product
	if((vFront * vTest) > 0.0)
		return false;

	return true;
}

template <typename T>
T COBB3<T>::GetMaximumExtent() const
{
	T max = m_Extents[0];
  int sorted[]={0,1,2};

  for(int i=2;i>=1;i--)
  {
	  for(int j=0;j<i;j++)
	  {
		  if(m_Extents[j] <= m_Extents[j+1]) {
			  int temp    = sorted[j];
        sorted[j]   = sorted[j+1];
        sorted[j+1] = temp;
		  }
	  }
  }

  max = (T)((m_Extents[sorted[0]] * m_vUVW[sorted[0]]) + (m_Extents[sorted[1]] * m_vUVW[sorted[1]])).mag();
	return max;
}

template <typename T>
T COBB3<T>::GetBoundingSphereRadius() const
{
  T radius = (T)((m_Extents[0] * m_vUVW[0]) + (m_Extents[1] * m_vUVW[1]) + (m_Extents[2] * m_vUVW[2])).mag();
	return radius;
}

template <typename T>
CAABB3<T> COBB3<T>::GetAABB()
{
	return CAABB3<T>(m_vCenter,m_Extents);
}

template <typename T>
CVector3<T> COBB3<T>::GetVertex(int index) const
{
    CVector3<T> extAxis0 = m_Extents[0]*m_vUVW[0];
    CVector3<T> extAxis1 = m_Extents[1]*m_vUVW[1];
    CVector3<T> extAxis2 = m_Extents[2]*m_vUVW[2];

    switch(index)
    {
    case 0:
      return m_vCenter - extAxis0 - extAxis1 - extAxis2;
      break;
    case 1:
      return m_vCenter + extAxis0 - extAxis1 - extAxis2;
      break;
    case 2:
      return m_vCenter + extAxis0 + extAxis1 - extAxis2;
      break;
    case 3:
      return m_vCenter - extAxis0 + extAxis1 - extAxis2;
      break;
    case 4:
      return m_vCenter - extAxis0 - extAxis1 + extAxis2;
      break;
    case 5:
      return m_vCenter + extAxis0 - extAxis1 + extAxis2;
      break;
    case 6:
      return m_vCenter + extAxis0 + extAxis1 + extAxis2;
      break;
    case 7:
      return m_vCenter - extAxis0 + extAxis1 + extAxis2;
      break;
    default:
      std::cerr<<"Invalid vertex index error in COBB3<T>::GetVertex."<<std::endl;
      exit(0);
      break;
    }
}

template <typename T>
unsigned int COBB3<T>::ClassifyVertexOnSurface(const CVector3<T> &pVertex) const
{
	//classify every vertex
	unsigned int iRegion=0;
  if(fabs(-m_Extents[0]-pVertex.x) < CMath<Real>::EPSILON4)
	{
		iRegion |= 0x01;
	}
	if(fabs(pVertex.x-m_Extents[0]) < CMath<Real>::EPSILON4)
	{
		iRegion |= 0x02;
	}
	if(fabs(-m_Extents[1]-pVertex.y) < CMath<Real>::EPSILON4)
	{
		iRegion |= 0x04;
	}
	if(fabs(pVertex.y-m_Extents[1]) < CMath<Real>::EPSILON4)
	{
		iRegion |= 0x08;
	}
	if(fabs(-m_Extents[2]-pVertex.z) < CMath<Real>::EPSILON4)
	{
		iRegion |= 0x10;
	}
	if(fabs(pVertex.z-m_Extents[2]) < CMath<Real>::EPSILON4)
	{
		iRegion |= 0x20;
	}
	return iRegion;
}

template <typename T>
unsigned int COBB3<T>::ClassifyVertex(const CVector3<T> &pVertex) const
{

	//classify every vertex
	unsigned int iRegion=0;
	if(pVertex.x < -m_Extents[0])
	{
		iRegion |= 0x01;
	}
	if(pVertex.x > m_Extents[0])
	{
		iRegion |= 0x02;
	}
	if(pVertex.y < -m_Extents[1])
	{
		iRegion |= 0x04;
	}
	if(pVertex.y > m_Extents[1])
	{
		iRegion |= 0x08;
	}
	if(pVertex.z < -m_Extents[2])
	{
		iRegion |= 0x10;
	}
	if(pVertex.z > m_Extents[2])
	{
		iRegion |= 0x20;
	}
	return iRegion;
}


template <typename T>
CVector3<T> COBB3<T>::GetRegionVertex(unsigned int iRegion) const
{
  CVector3<T> vVertex;
  for(unsigned int i=1;i<=3;i++)
  {
    unsigned int m = 1;
    //left shift
    m <<= 2*(i-1);
    if((iRegion & m) != 0)
      vVertex.m_dCoords[i-1]=-m_Extents[i-1];
    else
      vVertex.m_dCoords[i-1]=m_Extents[i-1];
  }
  return vVertex;
}

template <typename T>
CSegment3<T> COBB3<T>::GetRegionEdge(unsigned int iRegion) const
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

  //get the vertex corresponding to code c1
  CVector3<T> vA = GetRegionVertex(c1);
  //get the vertex corresponding to code c2
  CVector3<T> vB = GetRegionVertex(c2);
  
  return CSegment3<T>(vA,vB);
}

template <typename T>
void COBB3<T>::GetFacesAtEdge(unsigned int iRegion, unsigned int faces[2]) const
{
  int j=0;
  for(int i=0;i<=5;i++)
  {
    unsigned int m1 = 1;
    m1<<=i;
    if(iRegion & m1)
    {
      faces[j++]=m1;
    }
  }
}

template <typename T>
CRectangle3<T> COBB3<T>::GetRegionFace(unsigned int iRegion) const
{

  CRectangle3<T> rec;
  CVector3<T> vAxes[3] = {CVector3<T>(1,0,0),CVector3<T>(0,1,0),CVector3<T>(0,0,1)};
  CVector3<T> extAxis0 = m_Extents[0] * vAxes[0];
  CVector3<T> extAxis1 = m_Extents[1] * vAxes[1];
  CVector3<T> extAxis2 = m_Extents[2] * vAxes[2];

  switch(iRegion)
  {
    case 1:
      rec.m_vCenter= - extAxis0;
      rec.m_vUV[0]=vAxes[1];
      rec.m_vUV[1]=vAxes[2];
      rec.m_Extents[0]=m_Extents[1];
      rec.m_Extents[1]=m_Extents[2];
      break;
    case 2:
      rec.m_vCenter= extAxis0;
      rec.m_vUV[0]=vAxes[1];
      rec.m_vUV[1]=vAxes[2];
      rec.m_Extents[0]=m_Extents[1];
      rec.m_Extents[1]=m_Extents[2];
      break;
    case 4:
      rec.m_vCenter= - extAxis1;
      rec.m_vUV[0]=vAxes[0];
      rec.m_vUV[1]=vAxes[2];
      rec.m_Extents[0]=m_Extents[0];
      rec.m_Extents[1]=m_Extents[2];
      break;
    case 8:
      rec.m_vCenter= extAxis1;
      rec.m_vUV[0]=vAxes[0];
      rec.m_vUV[1]=vAxes[2];
      rec.m_Extents[0]=m_Extents[0];
      rec.m_Extents[1]=m_Extents[2];
      break;
    case 16:
      rec.m_vCenter= - extAxis2;
      rec.m_vUV[0]=vAxes[0];
      rec.m_vUV[1]=vAxes[1];
      rec.m_Extents[0]=m_Extents[0];
      rec.m_Extents[1]=m_Extents[1];
      break;
    case 32:
      rec.m_vCenter= extAxis2;
      rec.m_vUV[0]=vAxes[0];
      rec.m_vUV[1]=vAxes[1];
      rec.m_Extents[0]=m_Extents[0];
      rec.m_Extents[1]=m_Extents[1];
      break;
  }
  return rec;
}

template <typename T>
CVector3<T> COBB3<T>::GetFaceNormal(unsigned int iRegion) const
{
  CVector3<T> vNormal;
  switch(iRegion)
  {
    case 1:
      vNormal =  -m_vUVW[0];
      break;
    case 2:
      vNormal =  m_vUVW[0];
      break;
    case 4:
      vNormal =  -m_vUVW[1];
      break;
    case 8:
      vNormal =  m_vUVW[1];
      break;
    case 16:
      vNormal =  -m_vUVW[2];
      break;
    case 32:
      vNormal =  m_vUVW[2];
      break;
  }
  return vNormal;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class COBB3<Real>;
//----------------------------------------------------------------------------

}