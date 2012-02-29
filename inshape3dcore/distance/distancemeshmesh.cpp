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
#include "distancemeshmesh.h"
#include <perftimer.h>

namespace i3d {

template <typename T>
CDistanceMeshMesh<T>::CDistanceMeshMesh() 
{

}

template <typename T>
CDistanceMeshMesh<T>::~CDistanceMeshMesh() 
{

}

template <typename T>
T CDistanceMeshMesh<T>::ComputeDistanceSqr()
{
  return T(0);
}

template <typename T>
T CDistanceMeshMesh<T>::ComputeDistance()
{
  return T(0);
}

template <typename T>
T CDistanceMeshMesh<T>::ComputeDistanceEps(T eps)
{

   /* top-down traverse the tree
     check the dist(currentNode, Node) against eps
     if dist(currentNode, Node) < esp
       if( !currentNode->isLeaf )
         expandNode
       else
         check triangles in currentNode
         store normals and closest points in vector
     
     return minDist,penetration,contact points, normals
  */

  double dTimeTraverse=0.0;
  double dTimeIntersection=0.0;
  CPerfTimer timer0;
  T eps2=eps*eps;
  std::list<CBoundingVolumeNode3<CAABB3<T>,T,CTraits>* > nodes;
  typename std::list<CBoundingVolumeNode3<CAABB3<T>,T,CTraits>* >::iterator liter;
  m_dEps = eps;

  int depth = m_pBVH0->GetDepth();
  std::vector<CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *> leaves0 = m_pBVH0->GetNodesLevel(depth);
  std::vector<CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *> leaves1 = m_pBVH1->GetNodesLevel(0);

  for(int i=0;i<leaves0.size();i++)
  {
    CAABB3<T> box0 = leaves0[i]->m_BV; // GetAABB();
    for(int j=0;j<leaves1.size();j++)
    {
      //calculate distance between the AABBs

      CAABB3<T> box1 = leaves1[j]->m_BV;

      T sqrDist=0;
      CVector3<T> vD;
      for(int k=0;k<3;k++)
      {
        if(box1.m_Verts[1].m_dCoords[k] < box0.m_Verts[0].m_dCoords[k])
        {
          vD.m_dCoords[k]= box1.m_Verts[1].m_dCoords[k] - box0.m_Verts[0].m_dCoords[k];
        }
        else if(box1.m_Verts[0].m_dCoords[k] > box0.m_Verts[1].m_dCoords[k])
        {
          vD.m_dCoords[k]= box1.m_Verts[0].m_dCoords[k] - box0.m_Verts[1].m_dCoords[k];
        }
        else
        {
          vD.m_dCoords[k]=0;
        }
        sqrDist+=vD.m_dCoords[k]*vD.m_dCoords[k];
      }//end for k
      //compute distance between bounding volumes
      if(sqrDist < eps2)
      {
        //add the pair of nodes
        pairs.push_back(std::pair<CBoundingVolumeNode3<CAABB3<T>,T,CTraits>*,
                                  CBoundingVolumeNode3<CAABB3<T>,T,CTraits>* >(leaves0[i],leaves1[j]));
      }//end if
    }//end j
  }//end i

  return T(0);

//    typename std::list<CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *>::iterator liter = leaves.begin();

//    T mindist = CMath<T>::MAXREAL;
//    int minindex=-1;
//    CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *node = *liter;
//    CVector3<T> normal;
//    CVector3<T> contactpoint;
//
//    for(;liter!=leaves.end();liter++)
//    {
//
//      CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *node = *liter;
//
//      for(int k=0;k<node->m_Traits.m_vTriangles.size();k++)
//      {
//        CTriangle3<T> &tri3 = node->m_Traits.m_vTriangles[k];       
//        CDistancePointTriangle<T> distPointTri(tri3,m_Sphere.Center());
//        T dist = distPointTri.ComputeDistance() - m_Sphere.Radius();
//        CVector3<T> vNormal = distPointTri.m_vClosestPoint1 - distPointTri.m_vClosestPoint0;
//        CVector3<T> vCP = (distPointTri.m_vClosestPoint0+distPointTri.m_vClosestPoint1)*0.5;
//        vNormal.Normalize();
//
//        if(dist < mindist)
//        {
//          mindist=dist;
//          minindex=k;
//          normal=vNormal;
//          contactpoint=vCP;
//        }
//
//      }//end for k
//
//      if(mindist < m_dEps)
//      {
//        m_vPoint.push_back(contactpoint);
//        m_vNormals.push_back(normal);
//      }
//
//    }//end for liter
//
//  }//end if

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceMeshMesh<Real>;


}
