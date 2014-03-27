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
#ifndef DISTANCEMESHPOINT_H
#define DISTANCEMESHPOINT_H



//===================================================
//                     INCLUDES
//===================================================
#include <distance.h>
#include <sphere.h>
#include <boundingvolumetree3.h>
#include <3dmodel.h>
#include <subdivisioncreator.h>
#include <vector>
#include <list>
#include <traits.h>

namespace i3d {

/**
 * @brief The result of a B&B search stores a node of the BVH tree and a triangle 
 * 
 */  
template <typename T>
class CDistanceMeshPointResult {
  
public:
  /**
   * Standard Contructor
   */
  CDistanceMeshPointResult() {};
  
  /**
   * Pointer to the node in the tree that contains the closest point
   */
  CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *pNode;
  
  /**
   * Pointer to the tree representation of the mesh
   */
  CBoundingVolumeTree3<CAABB3<T>,T,CTraits,CSubdivisionCreator> *m_pBVH;    
  
  /**
   * ID of the triangle in the mesh
   */
  int iTriangleID;
  
  /**
   * The closest point on the mesh
   */
  CVector3<T> m_vClosestPoint;  
  
};
  
/**
* @brief Computes the distance between a mesh and a point
*
* Computes the distance between tri-mesh and a point
*
*/   
template <typename T>
class CDistanceMeshPoint : public CDistance<T> {

public: 

  /**
   * Standard Contructor
   */ 
  CDistanceMeshPoint();

  CDistanceMeshPoint(CBoundingVolumeTree3<CAABB3<T>,T,CTraits,CSubdivisionCreator> *pBVH, const CVector3<T> &vQuery) : m_vQuery(vQuery), m_pBVH(pBVH)
  {
    m_Res.m_pBVH = m_pBVH;
  }; 

  /**
   * Destructor
   */ 
  ~CDistanceMeshPoint(); 

  /**
   *
   */  
  T ComputeDistanceSqr();

  T ComputeDistance();

  T ComputeDistanceEps(T eps);

  T ComputeDistanceEpsNaive(T eps);
  
  T ComputeDistanceCo(T beta);

  T ComputeDistanceCoSqr(T beta);

  /**
   * Distance tolerance for triangles/points to be included in the result
   */    
  T m_dEps;

  /**
   * The query point
   */
  CVector3<T> m_vQuery;
  
  /**
   * Pointer to the tree representation of the mesh
   */
  CBoundingVolumeTree3<CAABB3<T>,T,CTraits,CSubdivisionCreator> *m_pBVH;  

  /**
   * Structure to store the result of the distance computation
   */
  CDistanceMeshPointResult<T> m_Res;    
  
private:
  
  /**
   * Traverse the tree representation
   */
  void Traverse(CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *pNode);

  /**
   * Storage for the triangles of the mesh
   */
  std::vector< CTriangle3<T> > m_vTriangles;

  /**
   * Storage for identified leaf node canditates
   */
  std::list<CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *> leaves;
  
  double distchecks;
  
  double adding;
  
  int    ndistchecks;

};

}
#endif
