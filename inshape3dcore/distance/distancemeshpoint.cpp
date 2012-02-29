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
#include "distancemeshpoint.h"
#include <traits.h>
#include <list>
#include <perftimer.h>
#include <distancetriangle.h>
#include <limits>

namespace i3d {

template <typename T>
CDistanceMeshPoint<T>::CDistanceMeshPoint() 
{

}

template <typename T>
CDistanceMeshPoint<T>::~CDistanceMeshPoint() 
{

}

template <typename T>
T CDistanceMeshPoint<T>::ComputeDistanceSqr()
{
  
  //variable declarations and initialisations
  //=========================================
  //helper variable we initialize it with the maximum possible value
  Real d = CMath<T>::MAXREAL;

  //further helper variables
  Real lowerBound  = CMath<T>::MAXREAL;
  Real upperBound  = -CMath<T>::MAXREAL;

  //In this variable we save the node of the BVH that
  //is located closest to the query point
  CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *pBest = NULL;

  //we need to count how many leaves of the
  //BVH we have in our list
  int nLeafCount = 0;

  //the list we need for the Breadth first search
  //in the tree data structure
  std::list<CBoundingVolumeNode3<CAABB3<T>,T,CTraits>* > lBFS;
  typename std::list<CBoundingVolumeNode3<CAABB3<T>,T,CTraits>* >::iterator lIter;
  
  //initialize this list with the children of the root
  for(int i=0;i< m_pBVH->GetNumChildren();i++)
    lBFS.push_back(m_pBVH->GetChild(i));

  //get the current size of the list
  int vSize = (int)lBFS.size();

  //* loop until there are only leaves in the list */
  while(vSize != nLeafCount)
  {

    //each time initialize with zeros
    nLeafCount = 0;
    int j = 0;

    //each time set this to the maximum value
    lowerBound = CMath<T>::MAXREAL;

    //a auxilliary array so that we dont have to
    //calculate these values multiple times
    T *dLowerBounds = new T[vSize];

    /* find best upper bound */
    for(lIter = lBFS.begin(); lIter != lBFS.end(); lIter++)
    {
      CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *pNode = *lIter;
      dLowerBounds[j] = pNode->GetLowerBound(m_vQuery);
      if(lowerBound > dLowerBounds[j])
      {
        lowerBound = dLowerBounds[j];
        pBest = pNode;
      }//end if
      j++;
    }//end for

    /* get upper bound for best element */
    upperBound = pBest->GetUpperBound(m_vQuery);
    
    //now we check every element if we can prune
    //it or if it has to remain
    lIter = lBFS.begin();
    for(int i = 0; i < vSize; i++)
    {
      //get the current element
      CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *pNode = *lIter;

      //if the current element is the best element
      //we replace it by its successors and we go on
      if(pNode == pBest)
      {
        //if we have reached the leaf
        //level no more refinement is possible
        if(!pNode->IsLeaf())
        {
          lBFS.push_back(pNode->m_Children[0]);
          lBFS.push_back(pNode->m_Children[1]);
          lIter = lBFS.erase(lIter);
          continue;
        }//end if
        //the node is a leaf, so we increase the leaf count
        else
        {
          nLeafCount++;
          lIter++;
          continue;
        }//end else
      }//end if

      //we check if our upper bound on the distance
      //is larger than the lower bound of the current node
      //If the lower bound of the current node is smaller
      //then we refine it
      if(upperBound > dLowerBounds[i])
      {
        //is the node a leaf, then
        // it can not be refined...
        if(!pNode->IsLeaf())
        {
          lBFS.push_back(pNode->m_Children[0]);
          lBFS.push_back(pNode->m_Children[1]);
          lIter = lBFS.erase(lIter);
        }//end if
        else
        {
          nLeafCount++;
          lIter++;
        }
      }//end if
      //the node's lower bound is larger than
      //the best upper bound, so it can be
      //pruned away
      else
      {
        //std::cout<<"this should not happen"<<std::endl;
        lIter = lBFS.erase(lIter);
      }//end else

    }//end for

    //update the the current size of the list
    vSize = (int)lBFS.size();

    //delete the auxilliary array, so we dont make
    //a memory leak
    delete[] dLowerBounds;
  }//end while

  //std::cout<<"leafcount: "<<nLeafCount<<std::endl;
  m_Res.pNode = pBest;
  //get all the triangles contained in the best node
  T mindist = CMath<T>::MAXREAL;
  for(int k=0;k<pBest->m_Traits.m_vTriangles.size();k++)
  {
    CTriangle3<T> &tri3 = pBest->m_Traits.m_vTriangles[k];
    CDistancePointTriangle<T> distPointTri(tri3,m_vQuery);
    T dist = distPointTri.ComputeDistance();
    if(dist < mindist)
    {
      mindist=dist;
      m_Res.iTriangleID = k;
    }
  }//end for k

  for(lIter=lBFS.begin();lIter!=lBFS.end();lIter++)
  {
    CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *node = *lIter;

    if(node == pBest)
      continue;

    for(int k=0;k<node->m_Traits.m_vTriangles.size();k++)
    {
      CTriangle3<T> &tri3 = node->m_Traits.m_vTriangles[k];
      CDistancePointTriangle<T> distPointTri(tri3,m_vQuery);
      T dist = distPointTri.ComputeDistance();
      if(dist < mindist)
      {
        mindist=dist;
        m_Res.pNode = node; 
        m_Res.iTriangleID = k;        
      }
    }//end for k
  }//end for liter

	//finally return the square root of the distance
  return T(mindist);
}

template <typename T>
T CDistanceMeshPoint<T>::ComputeDistance()
{
  return ComputeDistanceSqr();
}

template <typename T>
T CDistanceMeshPoint<T>::ComputeDistanceEps(T eps)
{
  return T(0);
}

template <typename T>
void CDistanceMeshPoint<T>::Traverse(CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *pNode)
{
}

template <typename T>
T CDistanceMeshPoint<T>::ComputeDistanceEpsNaive(T eps)
{
  return T(0);
}

template <typename T>
T CDistanceMeshPoint<T>::ComputeDistanceCo(T beta)
{
  return ComputeDistanceCoSqr(beta);
}

template <typename T>
T CDistanceMeshPoint<T>::ComputeDistanceCoSqr(T beta)
{
  
  //variable declarations and initialisations
  //=========================================
  //helper variable we initialize it with the maximum possible value
  Real d = CMath<T>::MAXREAL;

  //further helper variables
  Real lowerBound  = CMath<T>::MAXREAL;
  Real upperBound  = -CMath<T>::MAXREAL;

  //In this variable we save the node of the BVH that
  //is located closest to the query point
  CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *pBest = NULL;

  //we need to count how many leaves of the
  //BVH we have in our list
  int nLeafCount = 0;

  //the list we need for the Breadth first search
  //in the tree data structure
  std::list<CBoundingVolumeNode3<CAABB3<T>,T,CTraits>* > lBFS;
  typename std::list<CBoundingVolumeNode3<CAABB3<T>,T,CTraits>* >::iterator lIter;

  //initialize this list with the children of the root
  for(int i=0;i< m_pBVH->GetNumChildren();i++)
    lBFS.push_back(m_pBVH->GetChild(i));

  //get the current size of the list
  int vSize = (int)lBFS.size();

  //* loop until there are only leaves in the list */
  while(vSize != nLeafCount)
  {

    //each time initialize with zeros
    nLeafCount = 0;
    int j = 0;

    //each time set this to the maximum value
    lowerBound = CMath<T>::MAXREAL;

    //a auxilliary array so that we dont have to
    //calculate these values multiple times
    T *dLowerBounds = new T[vSize];

    /* find best upper bound */
    for(lIter = lBFS.begin(); lIter != lBFS.end(); lIter++)
    {
      CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *pNode = *lIter;
      dLowerBounds[j] = pNode->GetLowerBound(m_vQuery);
      if(lowerBound > dLowerBounds[j])
      {
        lowerBound = dLowerBounds[j];
        pBest = pNode;
      }//end if
      j++;
    }//end for

    /* get upper bound for best element */
    T alpha = pBest->GetUpperBound(m_vQuery);
    upperBound = (beta > alpha) ? alpha : beta;
    
    //now we check every element if we can prune
    //it or if it has to remain
    lIter = lBFS.begin();
    for(int i = 0; i < vSize; i++)
    {
      //get the current element
      CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *pNode = *lIter;

      //if the current element is the best element
      //we replace it by its successors and we go on
      if(pNode == pBest)
      {
        //if we have reached the leaf
        //level no more refinement is possible
        if(!pNode->IsLeaf())
        {
          lBFS.push_back(pNode->m_Children[0]);
          lBFS.push_back(pNode->m_Children[1]);
          lIter = lBFS.erase(lIter);
          continue;
        }//end if
        //the node is a leaf, so we increase the leaf count
        else
        {
          nLeafCount++;
          lIter++;
          continue;
        }//end else
      }//end if

      //we check if our upper bound on the distance
      //is larger than the lower bound of the current node
      //If the lower bound of the current node is smaller
      //then we refine it
      if(upperBound > dLowerBounds[i])
      {
        //is the node a leaf, then
        // it can not be refined...
        if(!pNode->IsLeaf())
        {
          lBFS.push_back(pNode->m_Children[0]);
          lBFS.push_back(pNode->m_Children[1]);
          lIter = lBFS.erase(lIter);
        }//end if
        else
        {
          nLeafCount++;
          lIter++;
        }
      }//end if
      //the node's lower bound is larger than
      //the best upper bound, so it can be
      //pruned away
      else
      {
        //std::cout<<"this should not happen"<<std::endl;
        lIter = lBFS.erase(lIter);
      }//end else

    }//end for

    //update the the current size of the list
    vSize = (int)lBFS.size();

    //delete the auxilliary array, so we dont make
    //a memory leak
    delete[] dLowerBounds;
  }//end while

  //std::cout<<"leafcount: "<<nLeafCount<<std::endl;
  m_Res.pNode = pBest;
  //get all the triangles contained in the best node
  T mindist = CMath<T>::MAXREAL;
  for(int k=0;k<pBest->m_Traits.m_vTriangles.size();k++)
  {
    CTriangle3<T> &tri3 = pBest->m_Traits.m_vTriangles[k];
    CDistancePointTriangle<T> distPointTri(tri3,m_vQuery);
    T dist = distPointTri.ComputeDistance();
    if(dist < mindist)
    {
      mindist=dist;
      m_Res.iTriangleID = k;      
    }
  }//end for k

  for(lIter=lBFS.begin();lIter!=lBFS.end();lIter++)
  {
    CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *node = *lIter;

    if(node == pBest)
      continue;

    for(int k=0;k<node->m_Traits.m_vTriangles.size();k++)
    {
      CTriangle3<T> &tri3 = node->m_Traits.m_vTriangles[k];
      CDistancePointTriangle<T> distPointTri(tri3,m_vQuery);
      T dist = distPointTri.ComputeDistance();
      if(dist < mindist)
      {
        mindist=dist;
        m_Res.pNode = node;
        m_Res.iTriangleID = k;        
      }
    }//end for k
  }//end for liter

  //finally return the square root of the distance
  return T(mindist);
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceMeshPoint<Real>;

}
