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
#ifndef DISTANCECONVEXCONVEXGJK_H
#define DISTANCECONVEXCONVEXGJK_H



//===================================================
//                     INCLUDES
//===================================================
#include <distance.h>
#include <convexshape.h>
#include <transform.h>
#include <vector>

namespace i3d {

/**
 * @brief Helper class that stores a simplex in the GJK-Algorithm
 */
template <class T>
class CSimplexDescriptorGjk {
  
public:
  
  CSimplexDescriptorGjk()
  {
    memset(m_dBarycentricCoordinates,0,3*sizeof(T));
    m_iNumVertices = 0;
  };
  
  ~CSimplexDescriptorGjk(){};

  CSimplexDescriptorGjk(const CSimplexDescriptorGjk &copy)
  {
    m_iNumVertices                = copy.m_iNumVertices;
    memcpy(m_dBarycentricCoordinates,copy.m_dBarycentricCoordinates,3*sizeof(T));
    memcpy(m_vSimplexVertices,copy.m_vSimplexVertices,4*sizeof(Vector3<T>));
    memcpy(m_vSupportVerticesA,copy.m_vSupportVerticesA,4*sizeof(Vector3<T>));
    memcpy(m_vSupportVerticesB,copy.m_vSupportVerticesB,4*sizeof(Vector3<T>));
  }

  /**
   * Adds a new support vertex for shape A to the current simplex
   */
  inline void AddSupportA(const Vector3<T> &vecA)
  {
    m_vSupportVerticesA[m_iNumVertices]=vecA;
  }
  
  /**
   * Adds a new support vertex for shape A to the current simplex
   */  
  inline void AddSupportB(const Vector3<T> &vecB)
  {
    m_vSupportVerticesB[m_iNumVertices]=vecB;
  }

  /**
   * Adds a new vertex to the simplex
   */    
  inline void AddSimplexVertex(const Vector3<T> &w)
  {
    m_vSimplexVertices[m_iNumVertices]=w;
  }

  /**
   * Returns the simplex vertex indexed by i
   */      
  inline Vector3<T>& GetSimplexVertex(int i)
  {
    return m_vSimplexVertices[i];
  }

  /**
   * Getter method for the support vertices of shape A
   */        
  inline Vector3<T> getSupportA(int i) {return m_vSupportVerticesA[i];};

  /**
   * Getter method for the support vertices of shape B
   */          
  inline Vector3<T> getSupportB(int i) {return m_vSupportVerticesB[i];};

  /**
   * Setter method for the current vertex cound
   */            
  inline void SetVertexCount(int count) {m_iNumVertices=count;};
  
  /**
   * Getter method for the current vertex cound
   */              
  inline int  GetVertexCount() {return m_iNumVertices;};

  /**
   * Increment the current vertex cound
   */                
  inline void IncreaseVertexCount() {m_iNumVertices++;};
  

  /**
   * The current vertex count
   */
  int         m_iNumVertices;
  
  /**
   * Barycentric coordinates
   */
  T           m_dBarycentricCoordinates[3];
  
  /**
   * Vertices of the simplex
   */
  Vector3<T> m_vSimplexVertices[4];
  
  /**
   * Storage for the bookeeping of support vertices for shape A
   */  
  Vector3<T> m_vSupportVerticesA[4];
  
  /**
   * Storage for the bookeeping of support vertices for shape B
   */    
  Vector3<T> m_vSupportVerticesB[4];

};

/**
 * @brief Computes the distance between two convex shapes by the GJK-Algorithm
 */  
template <class T>
class CDistanceConvexConvexGjk : public CDistance<T> {

public: 

  /**
   * Standard constructor
   */
  CDistanceConvexConvexGjk();

  CDistanceConvexConvexGjk(const ConvexShape<T> &shape0, const ConvexShape<T> &shape1,
                                                   const Transformation<T> &transform0, const Transformation<T> &transform1);

  /**
   * Destructor
   */  
  ~CDistanceConvexConvexGjk();

  T ComputeDistanceSqr();
  
  T ComputeDistance();

  T ComputeDistanceMargin();
  
  T ComputeDistanceMarginSqr();

  /**
   * Set the tolerance for the algorithm to exit
   */  
  void SetEpsilion(T eps) {epsilon = eps;};

  /**
   * Set the maximum number of iterations
   */  
  void SetMaxIterations(int iterations) {maxIter = iterations;};
  
  /**
   * Stores a pointer to the first convex shape
   */  
  const ConvexShape<T> *m_pShape0;
  
  /**
   * Stores a pointer to the second convex shape
   */    
  const ConvexShape<T> *m_pShape1;  
  
  /**
   * Stores a pointer to the first shape's transformation
   */    
  const Transformation<T> *m_pTransform0;
  
  /**
   * Stores a pointer to the second shape's transformation
   */      
  const Transformation<T> *m_pTransform1;  
  
  /**
   * Closest point on the first shape
   */      
  using CDistance<T>::m_vClosestPoint0;
  
  /**
   * Closest point on the second shape
   */        
  using CDistance<T>::m_vClosestPoint1;   

  /**
   * Stores the current simplex
   */
  CSimplexDescriptorGjk<T> m_vSimplex;

  /**
   * The tolerance of the distance algorithm
   */
  T epsilon;
  
  /**
   * Maximum number of iterations in the algorithm
   */
  int maxIter;
  
private:
  //TODO: change brute force for all non-empty simplices search
  /**
   * Computes the smallest set of vertices needed to represent the current simplex
   */
  CSimplexDescriptorGjk<T> ComputeSmallestSet(CSimplexDescriptorGjk<T> simplex, Vector3<T> &v);
  
  /**
   * Computes the minimum distance of the current support vertex to the simplex
   */
  void                     ComputeClosestPoints();

};

}
#endif


