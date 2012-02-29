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
    memcpy(m_vSimplexVertices,copy.m_vSimplexVertices,4*sizeof(CVector3<T>));
    memcpy(m_vSupportVerticesA,copy.m_vSupportVerticesA,4*sizeof(CVector3<T>));
    memcpy(m_vSupportVerticesB,copy.m_vSupportVerticesB,4*sizeof(CVector3<T>));
  }

  inline void AddSupportA(const CVector3<T> &vecA)
  {
    m_vSupportVerticesA[m_iNumVertices]=vecA;
  }
  inline void AddSupportB(const CVector3<T> &vecB)
  {
    m_vSupportVerticesB[m_iNumVertices]=vecB;
  }

  inline void AddSimplexVertex(const CVector3<T> &w)
  {
    m_vSimplexVertices[m_iNumVertices]=w;
  }

  inline CVector3<T>& GetSimplexVertex(int i)
  {
    return m_vSimplexVertices[i];
  }

  inline CVector3<T> GetSupportA(int i) {return m_vSupportVerticesA[i];};

  inline CVector3<T> GetSupportB(int i) {return m_vSupportVerticesB[i];};

  inline void SetVertexCount(int count) {m_iNumVertices=count;};
  inline int  GetVertexCount() {return m_iNumVertices;};

  inline void IncreaseVertexCount() {m_iNumVertices++;};
  


  int         m_iNumVertices;
  T           m_dBarycentricCoordinates[3];
  CVector3<T> m_vSimplexVertices[4];
  CVector3<T> m_vSupportVerticesA[4];
  CVector3<T> m_vSupportVerticesB[4];

};

/**
 * @brief Computes the distance between two convex shapes by the GJK-Algorithm
 */  
template <class T>
class CDistanceConvexConvexGjk : public CDistance<T> {

public: 

  CDistanceConvexConvexGjk();

  CDistanceConvexConvexGjk(const CConvexShape<T> &shape0, const CConvexShape<T> &shape1,
                                                   const CTransform<T> &transform0, const CTransform<T> &transform1);

  ~CDistanceConvexConvexGjk();

  T ComputeDistanceSqr();
  T ComputeDistance();

  T ComputeDistanceMargin();
  T ComputeDistanceMarginSqr();

  void SetEpsilion(T eps) {epsilon = eps;};
  void SetMaxIterations(int iterations) {maxIter = iterations;};
  
  const CConvexShape<T> *m_pShape0;
  const CConvexShape<T> *m_pShape1;  
  
  const CTransform<T> *m_pTransform0;
  const CTransform<T> *m_pTransform1;  

  //the simplex in the gjk
  //simplexvariable
  
  using CDistance<T>::m_vClosestPoint0;
  using CDistance<T>::m_vClosestPoint1;   

  CSimplexDescriptorGjk<T> m_vSimplex;

  T epsilon;
  int maxIter;
  
private:
  //TODO: change brute force for all non-empty simplices search
  CSimplexDescriptorGjk<T> ComputeSmallestSet(CSimplexDescriptorGjk<T> simplex, CVector3<T> &v);
  void                     ComputeClosestPoints();

};

}
#endif


