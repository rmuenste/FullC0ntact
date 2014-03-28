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
#include "distancemodelplane.h"
#include <traits.h>
#include <list>
#include <perftimer.h>

namespace i3d {

template <typename T>
CDistanceModelPlane<T>::CDistanceModelPlane() 
{

}

template <typename T>
CDistanceModelPlane<T>::~CDistanceModelPlane() 
{

}

template <typename T>
T CDistanceModelPlane<T>::ComputeDistanceSqr()
{
  return T(0);
}

template <typename T>
T CDistanceModelPlane<T>::ComputeDistance()
{
  return T(0);
}

template <typename T>
T CDistanceModelPlane<T>::ComputeDistanceEps(T eps)
{
  /* top-down traverse the tree
     check the dist(currentNode,Plane) against eps
     if dist(currentNode,Plane) < esp
       if( !currentNode->isLeaf )
         expandNode
       else
         check triangles in currentNode
         store normals and closest points in vector
     
     return minDist,penetration,contact points, normals
  */
  distchecks=0;
  ndistchecks=0;
  adding=0;
  double dTimeTraverse=0.0;
  double dTimeIntersection=0.0;
  CPerfTimer timer0;


  std::list<CBoundingVolumeNode3<AABB3<T>,T,CTraits>* > nodes;
  typename std::list<CBoundingVolumeNode3<AABB3<T>,T,CTraits>* >::iterator liter;
  m_dEps = eps;

  //early out test
  for(int i=0;i< m_pBVH->GetNumChildren();i++)
  {
    //compute distance AABB-Plane
    CBoundingVolumeNode3<AABB3<T>,T,CTraits> *pNode = m_pBVH->GetChild(i);

    //project point on plane
    CVector3<T> PQ = pNode->GetCenter() - m_pPlane->m_vOrigin;
    T sdistCenterPlane = m_pPlane->m_vNormal * PQ;
    CVector3<T> closestPoint = pNode->GetCenter() - sdistCenterPlane * m_pPlane->m_vNormal;

    //calculate distance from point to AABB surface 
    T distPlaneBox = pNode->m_BV.minDistance(closestPoint);
    if(distPlaneBox > eps)
      return T(-1.0);
    else
      nodes.push_back(pNode);
  }

  timer0.Start();
  for(liter=nodes.begin();liter!=nodes.end();liter++)
  {
    CBoundingVolumeNode3<AABB3<T>,T,CTraits> *pNode = *liter;
    Traverse(pNode);
  }

  dTimeTraverse=timer0.GetTime();
  //std::cout<<"Time traversal: "<<dTimeTraverse<<std::endl;
  timer0.Start();
  //check the size of the triangle vector
  if(!leaves.empty())
  {
    //compute dist(plane,triangles)
    //fill normals and points
    typename std::list<CBoundingVolumeNode3<AABB3<T>,T,CTraits> *>::iterator liter = leaves.begin();
    for(;liter!=leaves.end();liter++)
    {
      CBoundingVolumeNode3<AABB3<T>,T,CTraits> *node = *liter;

      for(int k=0;k<node->m_Traits.m_vTriangles.size();k++)
      {
        Triangle3<T> &tri3 = node->m_Traits.m_vTriangles[k];
        VECTOR3 vPoint = tri3.Get(0);
        Real dist = (vPoint - m_pPlane->m_vOrigin) * m_pPlane->m_vNormal; //mindist point-plane vertex 0

        for(int j=1;j<3;j++)
        {
          Real d = (tri3.Get(j) - m_pPlane->m_vOrigin) * m_pPlane->m_vNormal; //mindist point-plane vertex 0
          if(d < dist)
          {
            dist = d;
            vPoint = tri3.Get(j);
          }
        }//end for j

        if(dist < m_dEps)
          m_vPoint.push_back(vPoint);
      }//end for k

    }//end for liter

  }//end if


  //if(!m_vTriangles.empty())
  //{
  //  //compute dist(plane,triangles)
  //  //fill normals and points
  //  typename std::vector<CTriangle3<T> >::iterator titer = m_vTriangles.begin();
  //  for(;titer!=m_vTriangles.end();titer++)
  //  {

  //    CTriangle3<T> &tri3 = *titer;
  //    VECTOR3 vPoint = tri3.Get(0);
  //    Real dist = (vPoint - m_pPlane->m_vOrigin) * m_pPlane->m_vNormal; //mindist point-plane vertex 0

  //    for(int j=1;j<3;j++)
  //    {
  //      Real d = (tri3.Get(j) - m_pPlane->m_vOrigin) * m_pPlane->m_vNormal; //mindist point-plane vertex 0
  //      if(d < dist)
  //      {
  //        dist = d;
  //        vPoint = tri3.Get(j);
  //      }
  //    }//end for j

  //    if(dist < m_dEps)
  //      m_vPoint.push_back(vPoint);
  //  }
  //}

  //dTimeTraverse=timer0.GetTime();
  //std::cout<<"Time intersection: "<<dTimeTraverse<<std::endl;
  //std::cout<<"Number of intersections: "<<m_vTriangles.size()<<std::endl;
  //std::cout<<"Dist checking in traversal: "<<distchecks<<std::endl;
  //std::cout<<"Number of Dist checking in traversal: "<<ndistchecks<<std::endl;
  //std::cout<<"adding: "<<adding<<std::endl;


  return T(0);
}

template <typename T>
void CDistanceModelPlane<T>::Traverse(CBoundingVolumeNode3<AABB3<T>,T,CTraits> *pNode)
{
  
  //early out test
  CPerfTimer timer0;
  //compute distance AABB-Plane
  timer0.Start();
  //project point on plane
  CVector3<T> PQ = pNode->GetCenter() - m_pPlane->m_vOrigin;
  T sdistCenterPlane = m_pPlane->m_vNormal * PQ;
  CVector3<T> closestPoint = pNode->GetCenter() - sdistCenterPlane * m_pPlane->m_vNormal;

  //calculate distance from point to AABB surface 
  T distPlaneBox = pNode->m_BV.minDistance(closestPoint);

  //stop searching this branch
  if(distPlaneBox > m_dEps)
    return;

  distchecks+=timer0.GetTime();
  ndistchecks++;

  if(!pNode->IsLeaf())
  {
    Traverse(pNode->m_Children[0]);
    Traverse(pNode->m_Children[1]);
  }
  else
  {
    timer0.Start();
    leaves.push_back(pNode);
    ////add the triangles in the nodes to the result vector
    //typename std::vector< CTriangle3<T> >::iterator i = pNode->m_Traits.m_vTriangles.begin();
    //for(;i!=pNode->m_Traits.m_vTriangles.end();i++)
    //{
    //  m_vTriangles.push_back(*i);
    //}
    adding+=timer0.GetTime();
  }


}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceModelPlane<Real>;

}
