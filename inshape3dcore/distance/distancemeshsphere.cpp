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
#include "distancemeshsphere.h"
#include <traits.h>
#include <list>
#include <perftimer.h>
#include <distancetriangle.h>
#include <limits>

namespace i3d {

template <typename T>
CDistanceMeshSphere<T>::CDistanceMeshSphere() 
{

}

template <typename T>
CDistanceMeshSphere<T>::~CDistanceMeshSphere() 
{

}

template <typename T>
T CDistanceMeshSphere<T>::ComputeDistanceSqr()
{
  return T(0);
}

template <typename T>
T CDistanceMeshSphere<T>::ComputeDistance()
{
  return T(0);
}

template <typename T>
T CDistanceMeshSphere<T>::ComputeDistanceEps(T eps)
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
  T val;
  std::list<CBoundingVolumeNode3<AABB3<T>,T,CTraits>* > nodes;
  typename std::list<CBoundingVolumeNode3<AABB3<T>,T,CTraits>* >::iterator liter;
  m_dEps = eps;

  timer0.Start();
  //early out test
  for(int i=0;i< m_pBVH->GetNumChildren();i++)
  {
    //compute distance AABB-Plane
    CBoundingVolumeNode3<AABB3<T>,T,CTraits> *pNode = m_pBVH->GetChild(i);

    T myd = fabs(pNode->m_BV.minDistance(m_Sphere.getCenter())-m_Sphere.getRadius());

    //project point on plane
    bool inside = pNode->m_BV.isPointInside(m_Sphere.getCenter());
    if(inside)
      nodes.push_back(pNode);
    else if(fabs(pNode->m_BV.minDistance(m_Sphere.getCenter())-m_Sphere.getRadius()) < eps)
      nodes.push_back(pNode);
    else if(pNode->m_BV.minDistance(m_Sphere.getCenter()) < m_Sphere.getRadius())
      nodes.push_back(pNode);
  }

  if(nodes.empty())
  {
    dTimeIntersection=timer0.GetTime();
    //std::cout<<"Time intersection early out: "<<dTimeIntersection<<std::endl;
    return T(0);
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
    T mindist = CMath<T>::MAXREAL;
    int minindex=-1;
    CBoundingVolumeNode3<AABB3<T>,T,CTraits> *node = *liter;
    CVector3<T> normal;
    CVector3<T> contactpoint;

    for(;liter!=leaves.end();liter++)
    {

      CBoundingVolumeNode3<AABB3<T>,T,CTraits> *node = *liter;

      for(int k=0;k<node->m_Traits.m_vTriangles.size();k++)
      {
        Triangle3<T> &tri3 = node->m_Traits.m_vTriangles[k];       
        CDistancePointTriangle<T> distPointTri(tri3,m_Sphere.getCenter());
        T dist = distPointTri.ComputeDistance() - m_Sphere.getRadius();
        CVector3<T> vNormal = distPointTri.m_vClosestPoint1 - distPointTri.m_vClosestPoint0;
        CVector3<T> vCP = (distPointTri.m_vClosestPoint0+distPointTri.m_vClosestPoint1)*0.5;
        vNormal.Normalize();

        if(dist < mindist)
        {
          mindist=dist;
          minindex=k;
          normal=vNormal;
          contactpoint=vCP;
          val=mindist;
        }

      }//end for k

      if(mindist < m_dEps)
      {
        m_vPoint.push_back(contactpoint);
        m_vNormals.push_back(normal);
      }

    }//end for liter

  }//end if

  ////dTimeTraverse=timer0.GetTime();
  dTimeIntersection = timer0.GetTime();  
/*  std::cout<<"Time traversal: "<<dTimeTraverse<<std::endl;  
  std::cout<<"Time intersection: "<<dTimeIntersection<<std::endl;*/
  
  ////std::cout<<"Number of intersections: "<<m_vTriangles.size()<<std::endl;
  ////std::cout<<"Dist checking in traversal: "<<distchecks<<std::endl;
  ////std::cout<<"Number of Dist checking in traversal: "<<ndistchecks<<std::endl;
  ////std::cout<<"adding: "<<adding<<std::endl;

  return T(val);
}

template <typename T>
void CDistanceMeshSphere<T>::Traverse(CBoundingVolumeNode3<AABB3<T>,T,CTraits> *pNode)
{

  //if the point is in the node -> add
  //ifthe point is outside but closer than the radius -> add

  //project point on plane
  bool inside = pNode->m_BV.isPointInside(m_Sphere.getCenter());
  if(inside)
  {
    if(!pNode->IsLeaf())
    {
      Traverse(pNode->m_Children[0]);
      Traverse(pNode->m_Children[1]);
    }
    else
    {
      leaves.push_back(pNode);
    }
  }
  else if(pNode->m_BV.minDistance(m_Sphere.getCenter()) < m_Sphere.getRadius())
  {
    if(!pNode->IsLeaf())
    {
      Traverse(pNode->m_Children[0]);
      Traverse(pNode->m_Children[1]);
    }
    else
    {
      leaves.push_back(pNode);
    }
  }
  else if(fabs(pNode->m_BV.minDistance(m_Sphere.getCenter())-m_Sphere.getRadius()) < m_dEps)
  {
    if(!pNode->IsLeaf())
    {
      Traverse(pNode->m_Children[0]);
      Traverse(pNode->m_Children[1]);
    }
    else
    {
      leaves.push_back(pNode);
    }
  }
  else
    return;

}

template <typename T>
T CDistanceMeshSphere<T>::ComputeDistanceEpsNaive(T eps)
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

  timer0.Start();
  //early out test
  for(int i=0;i< m_pBVH->GetNumChildren();i++)
  {
    //compute distance AABB-Plane
    CBoundingVolumeNode3<AABB3<T>,T,CTraits> *pNode = m_pBVH->GetChild(i);

    T myd = fabs(pNode->m_BV.minDistance(m_Sphere.getCenter())-m_Sphere.getRadius());

    //project point on plane
    bool inside = pNode->m_BV.isPointInside(m_Sphere.getCenter());
    if(inside)
      nodes.push_back(pNode);
    else if(fabs(pNode->m_BV.minDistance(m_Sphere.getCenter())-m_Sphere.getRadius()) < eps)
      nodes.push_back(pNode);
    else if(pNode->m_BV.minDistance(m_Sphere.getCenter()) < m_Sphere.getRadius())
      nodes.push_back(pNode);
  }

  if(nodes.empty())
  {
    dTimeIntersection=timer0.GetTime();
    //std::cout<<"Time intersection early out: "<<dTimeIntersection<<std::endl;
    return T(0);
  }
  
  timer0.Start();
  T mindist = CMath<T>::MAXREAL;
  for(liter=nodes.begin();liter!=nodes.end();liter++)
  {

      int minindex=-1;
      CBoundingVolumeNode3<AABB3<T>,T,CTraits> *node = *liter;
      CVector3<T> normal;
      CVector3<T> contactpoint;

      for(int k=0;k<node->m_Traits.m_vTriangles.size();k++)
      {
        Triangle3<T> &tri3 = node->m_Traits.m_vTriangles[k];

        CDistancePointTriangle<T> distPointTri(tri3,m_Sphere.getCenter());
        T dist = distPointTri.ComputeDistance() - m_Sphere.getRadius();
        CVector3<T> vNormal = distPointTri.m_vClosestPoint1 - distPointTri.m_vClosestPoint0;
        CVector3<T> vCP = (distPointTri.m_vClosestPoint0+distPointTri.m_vClosestPoint1)*0.5;
        vNormal.Normalize();

        if(dist < mindist)
        {
          mindist=dist;
          minindex=k;
          normal=vNormal;
          contactpoint=vCP;
        }

      }//end for k

      if(mindist < m_dEps)
      {
        m_vPoint.push_back(contactpoint);
        m_vNormals.push_back(normal);
      }

  }//end for liter

  dTimeIntersection=timer0.GetTime();
  //std::cout<<"Time intersection: "<<dTimeIntersection<<std::endl;


  return T(mindist);
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceMeshSphere<Real>;

}
