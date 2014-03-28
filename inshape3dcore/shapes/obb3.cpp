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
OBB3<T>::OBB3 ()
{
}

template <typename T>
OBB3<T>::~OBB3 ()
{
}


template <typename T>
OBB3<T>::OBB3 (const CVector3<T>& center, const CVector3<T> axis[3], const T extent[3])
{
		center_ = center;
    uvw_[0] = axis[0];
    uvw_[1] = axis[1];
    uvw_[2] = axis[2];
    extents_[0] = extent[0];
    extents_[1] = extent[1];
    extents_[2] = extent[2];
}

template <typename T>
OBB3<T>::OBB3(const OBB3<T> &copy)
{
  center_ = copy.center_;
  uvw_[0] = copy.uvw_[0];
  uvw_[1] = copy.uvw_[1];
  uvw_[2] = copy.uvw_[2];
  extents_[0] = copy.extents_[0];
  extents_[1] = copy.extents_[1];
  extents_[2] = copy.extents_[2];
}



template <typename T>
OBB3<T>::OBB3 (const CVector3<T>& center, const CVector3<T>& axis0,
    const CVector3<T>& axis1, const CVector3<T>& axis2,
    const T extent0, const T extent1, const T extent2)
    :
    center_(center)
{
    uvw_[0] = axis0;
    uvw_[1] = axis1;
    uvw_[2] = axis2;
    extents_[0] = extent0;
    extents_[1] = extent1;
    extents_[2] = extent2;
}

template <typename T>
void OBB3<T>::computeVertices (CVector3<T> vertex[8]) const
{
    CVector3<T> extAxis0 = extents_[0]*uvw_[0];
    CVector3<T> extAxis1 = extents_[1]*uvw_[1];
    CVector3<T> extAxis2 = extents_[2]*uvw_[2];

    vertex[0] = center_ - extAxis0 - extAxis1 - extAxis2;
    vertex[1] = center_ + extAxis0 - extAxis1 - extAxis2;
    vertex[2] = center_ + extAxis0 + extAxis1 - extAxis2;
    vertex[3] = center_ - extAxis0 + extAxis1 - extAxis2;
    vertex[4] = center_ - extAxis0 - extAxis1 + extAxis2;
    vertex[5] = center_ + extAxis0 - extAxis1 + extAxis2;
    vertex[6] = center_ + extAxis0 + extAxis1 + extAxis2;
    vertex[7] = center_ - extAxis0 + extAxis1 + extAxis2;
}

template <typename T>
bool OBB3<T>::isPointInside (const CVector3<T> &vQuery) const
{

	CVector3<T> extAxis0 = extents_[0]*uvw_[0];
	CVector3<T> extAxis1 = extents_[1]*uvw_[1];
	CVector3<T> extAxis2 = extents_[2]*uvw_[2];

	CVector3<T> vTest;

	//point on top face
	CVector3<T> vMidTop = center_ + extAxis2;
	//normal vector
	CVector3<T> vTop = uvw_[2];
	//compute the test vector
	vTest = vQuery - vMidTop;
	//check sign of scalar product
	if((vTop * vTest) > 0.0)
		return false;

	//point on bottom face
	CVector3<T> vMidBot = center_ - extAxis2;
	//normal vector
	CVector3<T> vBot = -uvw_[2];
	//compute the test vector
	vTest = vQuery - vMidBot;
	//check sign of scalar product
	if((vBot * vTest) > 0.0)
		return false;

	//point on right face
	CVector3<T> vMidRight = center_ + extAxis0;
	//normal vector
	CVector3<T> vRight = uvw_[0];
	//compute the test vector
	vTest = vQuery - vMidRight;
	//check sign of scalar product
	if((vRight * vTest) > 0.0)
		return false;

	//point on left face
	CVector3<T> vMidLeft = center_ - extAxis0;
	//normal vector
	CVector3<T> vLeft = -uvw_[0];
	//compute the test vector
	vTest = vQuery - vMidLeft;
	//check sign of scalar product
	if((vLeft * vTest) > 0.0)
		return false;

	//point on back face
	CVector3<T> vMidBack = center_ + extAxis1;
	//normal vector
	CVector3<T> vBack = uvw_[1];
	//compute the test vector
	vTest = vQuery - vMidBack;
	//check sign of scalar product
	if((vBack * vTest) > 0.0)
		return false;

	//point on front face
	CVector3<T> vMidFront = center_ - extAxis1;
	//normal vector
	CVector3<T> vFront = -uvw_[1];
	//compute the test vector
	vTest = vQuery - vMidFront;
	//check sign of scalar product
	if((vFront * vTest) > 0.0)
		return false;

	return true;
}

template <typename T>
T OBB3<T>::getMaximumExtent() const
{
	T max = extents_[0];
  int sorted[]={0,1,2};

  for(int i=2;i>=1;i--)
  {
	  for(int j=0;j<i;j++)
	  {
		  if(extents_[j] <= extents_[j+1]) {
			  int temp    = sorted[j];
        sorted[j]   = sorted[j+1];
        sorted[j+1] = temp;
		  }
	  }
  }

  max = (T)((extents_[sorted[0]] * uvw_[sorted[0]]) + (extents_[sorted[1]] * uvw_[sorted[1]])).mag();
	return max;
}

template <typename T>
T OBB3<T>::getBoundingSphereRadius() const
{
  T radius = (T)((extents_[0] * uvw_[0]) + (extents_[1] * uvw_[1]) + (extents_[2] * uvw_[2])).mag();
	return radius;
}

template <typename T>
AABB3<T> OBB3<T>::getAABB()
{
  
  CVector3<T> minVec;
  CVector3<T>  maxVec;
  
  CVector3<T> vertices[8];
  computeVertices(vertices);
  
  for(int i = 0; i < 8; i++)
  {

    if(vertices[i].x < minVec.x)
    { //assign min index
      minVec.x = vertices[i].x;
    }

    if(vertices[i].x > maxVec.x)
    { //assign max index
      maxVec.x = vertices[i].x;
    }

    if(vertices[i].y < minVec.y)
    { //assign min index
      minVec.y = vertices[i].y;
    }

    if(vertices[i].y > maxVec.y)
    { //assign max index
      maxVec.y = vertices[i].y;
    }

    if(vertices[i].z < minVec.z)
    { //assign min index
      minVec.z = vertices[i].z;
    }

    if(vertices[i].z > maxVec.z)
    { //assign max index
      maxVec.z = vertices[i].z;
    }

  }//end for

  return AABB3<T>(minVec,maxVec);

}

template <typename T>
CVector3<T> OBB3<T>::getVertex(int index) const
{
    CVector3<T> extAxis0 = extents_[0]*uvw_[0];
    CVector3<T> extAxis1 = extents_[1]*uvw_[1];
    CVector3<T> extAxis2 = extents_[2]*uvw_[2];

    switch(index)
    {
    case 0:
      return center_ - extAxis0 - extAxis1 - extAxis2;
      break;
    case 1:
      return center_ + extAxis0 - extAxis1 - extAxis2;
      break;
    case 2:
      return center_ + extAxis0 + extAxis1 - extAxis2;
      break;
    case 3:
      return center_ - extAxis0 + extAxis1 - extAxis2;
      break;
    case 4:
      return center_ - extAxis0 - extAxis1 + extAxis2;
      break;
    case 5:
      return center_ + extAxis0 - extAxis1 + extAxis2;
      break;
    case 6:
      return center_ + extAxis0 + extAxis1 + extAxis2;
      break;
    case 7:
      return center_ - extAxis0 + extAxis1 + extAxis2;
      break;
    default:
      std::cerr<<"Invalid vertex index error in OBB3<T>::getVertex."<<std::endl;
      exit(0);
      break;
    }
}

template <typename T>
unsigned int OBB3<T>::classifyVertexOnSurface(const CVector3<T> &pVertex) const
{
	//classify every vertex
	unsigned int iRegion=0;
  if(fabs(-extents_[0]-pVertex.x) < CMath<Real>::EPSILON4)
	{
		iRegion |= 0x01;
	}
	if(fabs(pVertex.x-extents_[0]) < CMath<Real>::EPSILON4)
	{
		iRegion |= 0x02;
	}
	if(fabs(-extents_[1]-pVertex.y) < CMath<Real>::EPSILON4)
	{
		iRegion |= 0x04;
	}
	if(fabs(pVertex.y-extents_[1]) < CMath<Real>::EPSILON4)
	{
		iRegion |= 0x08;
	}
	if(fabs(-extents_[2]-pVertex.z) < CMath<Real>::EPSILON4)
	{
		iRegion |= 0x10;
	}
	if(fabs(pVertex.z-extents_[2]) < CMath<Real>::EPSILON4)
	{
		iRegion |= 0x20;
	}
	return iRegion;
}

template <typename T>
unsigned int OBB3<T>::classifyVertex(const CVector3<T> &pVertex) const
{

	//classify every vertex
	unsigned int iRegion=0;
	if(pVertex.x < -extents_[0])
	{
		iRegion |= 0x01;
	}
	if(pVertex.x > extents_[0])
	{
		iRegion |= 0x02;
	}
	if(pVertex.y < -extents_[1])
	{
		iRegion |= 0x04;
	}
	if(pVertex.y > extents_[1])
	{
		iRegion |= 0x08;
	}
	if(pVertex.z < -extents_[2])
	{
		iRegion |= 0x10;
	}
	if(pVertex.z > extents_[2])
	{
		iRegion |= 0x20;
	}
	return iRegion;
}


template <typename T>
CVector3<T> OBB3<T>::getRegionVertex(unsigned int iRegion) const
{
  CVector3<T> vVertex;
  for(unsigned int i=1;i<=3;i++)
  {
    unsigned int m = 1;
    //left shift
    m <<= 2*(i-1);
    if((iRegion & m) != 0)
      vVertex.m_dCoords[i-1]=-extents_[i-1];
    else
      vVertex.m_dCoords[i-1]=extents_[i-1];
  }
  return vVertex;
}

template <typename T>
CSegment3<T> OBB3<T>::getRegionEdge(unsigned int iRegion) const
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
  CVector3<T> vA = getRegionVertex(c1);
  //get the vertex corresponding to code c2
  CVector3<T> vB = getRegionVertex(c2);
  
  return CSegment3<T>(vA,vB);
}

template <typename T>
void OBB3<T>::getFacesAtEdge(unsigned int iRegion, unsigned int faces[2]) const
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
CRectangle3<T> OBB3<T>::getRegionFace(unsigned int iRegion) const
{

  CRectangle3<T> rec;
  CVector3<T> vAxes[3] = {CVector3<T>(1,0,0),CVector3<T>(0,1,0),CVector3<T>(0,0,1)};
  CVector3<T> extAxis0 = extents_[0] * vAxes[0];
  CVector3<T> extAxis1 = extents_[1] * vAxes[1];
  CVector3<T> extAxis2 = extents_[2] * vAxes[2];

  switch(iRegion)
  {
    case 1:
      rec.m_vCenter= - extAxis0;
      rec.m_vUV[0]=vAxes[1];
      rec.m_vUV[1]=vAxes[2];
      rec.m_Extents[0]=extents_[1];
      rec.m_Extents[1]=extents_[2];
      break;
    case 2:
      rec.m_vCenter= extAxis0;
      rec.m_vUV[0]=vAxes[1];
      rec.m_vUV[1]=vAxes[2];
      rec.m_Extents[0]=extents_[1];
      rec.m_Extents[1]=extents_[2];
      break;
    case 4:
      rec.m_vCenter= - extAxis1;
      rec.m_vUV[0]=vAxes[0];
      rec.m_vUV[1]=vAxes[2];
      rec.m_Extents[0]=extents_[0];
      rec.m_Extents[1]=extents_[2];
      break;
    case 8:
      rec.m_vCenter= extAxis1;
      rec.m_vUV[0]=vAxes[0];
      rec.m_vUV[1]=vAxes[2];
      rec.m_Extents[0]=extents_[0];
      rec.m_Extents[1]=extents_[2];
      break;
    case 16:
      rec.m_vCenter= - extAxis2;
      rec.m_vUV[0]=vAxes[0];
      rec.m_vUV[1]=vAxes[1];
      rec.m_Extents[0]=extents_[0];
      rec.m_Extents[1]=extents_[1];
      break;
    case 32:
      rec.m_vCenter= extAxis2;
      rec.m_vUV[0]=vAxes[0];
      rec.m_vUV[1]=vAxes[1];
      rec.m_Extents[0]=extents_[0];
      rec.m_Extents[1]=extents_[1];
      break;
  }
  return rec;
}

template <typename T>
CVector3<T> OBB3<T>::getFaceNormal(unsigned int iRegion) const
{
  CVector3<T> vNormal;
  switch(iRegion)
  {
    case 1:
      vNormal =  -uvw_[0];
      break;
    case 2:
      vNormal =  uvw_[0];
      break;
    case 4:
      vNormal =  -uvw_[1];
      break;
    case 8:
      vNormal =  uvw_[1];
      break;
    case 16:
      vNormal =  -uvw_[2];
      break;
    case 32:
      vNormal =  uvw_[2];
      break;
  }
  return vNormal;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class OBB3<Real>;
template class OBB3<float>;
//----------------------------------------------------------------------------

}
