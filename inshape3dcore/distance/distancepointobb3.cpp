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


#include "distancepointobb3.h"
#include <vector3.h>
#include <distancepointseg.h>
#include <distancepointrec.h>

namespace i3d {


template <class T>
CDistancePointObb3<T>::~CDistancePointObb3() 
{

}

template <class T>
T CDistancePointObb3<T>::ComputeDistance()
{
  return sqrt(ComputeDistanceSqr());
}

template <class T>
T CDistancePointObb3<T>::ComputeDistanceSqr()
{
  unsigned int iRegion;
  T minDistSqr = std::numeric_limits<T>::max();
  T distsqr;

  //store the result type
  int result=0;

  //transform the vertex into the box coordinate system
  Matrix3x3<T> mWorldModelT =  m_pTransform->getMatrix().GetTransposedMatrix();
  Vector3<T> vLocal = m_pPoint - m_pTransform->getOrigin();
  vLocal = mWorldModelT * vLocal;
  
  //classify the vertices with respect to the box
  iRegion = CDistanceTools<T>::ClassifyVertex(vLocal,*m_pBox);
  unsigned int iCode   = CDistanceTools<T>::GetRegionType(iRegion);
  if(iCode==CDistanceTools<T>::VERTEX)
  {
    //get the B1 vertex
    Vector3<T> vVertexBox = CDistanceTools<T>::GetRegionVertex(iRegion,*m_pBox);
    //Vertex-Vertex distance
    Vector3<T> vDiff = vLocal - vVertexBox;
    distsqr = vDiff * vDiff;
    minDistSqr = distsqr;
    result = 1;
    m_vClosestPoint1 = vVertexBox;
  }
  else if(iCode==CDistanceTools<T>::EDGE)
  {
    //get the edge of the box
    Segment3<T>seg = CDistanceTools<T>::GetRegionEdge(iRegion,*m_pBox);
    //calculate seg seg distance
    CDistancePointSeg<T> distPointSeg(vLocal,seg);
    T distEdgeSqr = distPointSeg.ComputeDistanceSqr();
    minDistSqr = distEdgeSqr;
    result = 2;
    //m_vClosestPoint0 is the point 
    m_vClosestPoint0 = vLocal;
    //m_vClosestPoint1 is the point on the edge region of the box
    m_vClosestPoint1 = distPointSeg.m_vClosestPoint1;
  }
  else if(iCode==CDistanceTools<T>::FACE)
  {
    //get the face
    Rectangle3<T> rec = CDistanceTools<T>::GetRegionFace(iRegion,*m_pBox);
    //calculate face seg distance
    CDistancePointRec<T> distPointRec(vLocal,rec);
    T distFaceSqr = distPointRec.ComputeDistanceSqr();
    minDistSqr = distFaceSqr;
    result = 3;
    //m_vClosestPoint0 is the point on the segment of Box0
    m_vClosestPoint0 = vLocal;
    //m_vClosestPoint1 is the point on the face(rectangle) of Box1
    m_vClosestPoint1 = distPointRec.m_vClosestPoint1;
  }

  //transform to world coordinates
  m_vClosestPoint0 = m_pPoint;
  m_vClosestPoint1 = m_pTransform->getMatrix() * m_vClosestPoint1 + m_pTransform->getOrigin();    
  if(result == CDistanceTools<T>::VERTEX)
  {
    //update the configuration
    m_ocConf.m_iConf            = CDistanceTools<T>::VERTEX;
    m_ocConf.m_vNormal          = m_vClosestPoint0 - m_vClosestPoint1;
    m_ocConf.m_vNormal.Normalize();
    m_ocConf.m_iFeature[0]      = CDistanceTools<T>::VERTEX;
    m_ocConf.m_iFeature[1]      = CDistanceTools<T>::VERTEX;
    m_ocConf.m_iFeatureIndex[0] = 0;
    m_ocConf.m_iFeatureIndex[1] = iRegion;
  }
  else if(result == CDistanceTools<T>::EDGE)
  {
    //update the configuration
    m_ocConf.m_iConf            = CDistanceTools<T>::EDGE;
    m_ocConf.m_vNormal          = m_vClosestPoint0 - m_vClosestPoint1;
    m_ocConf.m_vNormal.Normalize();
    m_ocConf.m_iFeature[0]      = CDistanceTools<T>::VERTEX;
    m_ocConf.m_iFeature[1]      = CDistanceTools<T>::EDGE;
    m_ocConf.m_iFeatureIndex[0] = 0;
    m_ocConf.m_iFeatureIndex[1] = iRegion;    
  }
  else if(result == CDistanceTools<T>::FACE)
  {
    //update the configuration
    m_ocConf.m_iConf            = CDistanceTools<T>::FACE;
    m_ocConf.m_vNormal          = CDistanceTools<T>::GetFaceNormal(iRegion,*m_pBox);
    //world transform the normal
    m_ocConf.m_vNormal          = m_pTransform->getMatrix() * m_ocConf.m_vNormal;
    m_ocConf.m_iFeature[0]      = CDistanceTools<T>::VERTEX;
    m_ocConf.m_iFeature[1]      = CDistanceTools<T>::FACE;
    m_ocConf.m_iFeatureIndex[0] = 0;
    m_ocConf.m_iFeatureIndex[1] = iRegion;        
  }
  else
  {
    
  }
  
  return minDistSqr;
}



//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistancePointObb3<Real>;
//----------------------------------------------------------------------------

}
