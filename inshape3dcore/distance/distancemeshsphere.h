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
#ifndef DISTANCEMESHSPHERE_H
#define DISTANCEMESHSPHERE_H



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
* @brief Computes the distance between a mesh and a sphere
*
* Computes the distance between tri-mesh and a sphere
*
*/   
template <typename T>
class CDistanceMeshSphere : public CDistance<T> {

public: 

  CDistanceMeshSphere();

  CDistanceMeshSphere(CBoundingVolumeTree3<AABB3<T>,T,CTraits,CSubdivisionCreator> *pBVH, const Sphere<T> &sphere) : m_Sphere(sphere), m_pBVH(pBVH) {}; 

  ~CDistanceMeshSphere(); 

	T ComputeDistanceSqr();

	T ComputeDistance();

	T ComputeDistanceEps(T eps);

  T ComputeDistanceEpsNaive(T eps);

  std::vector<CVector3<T> > m_vPoint;

  std::vector<CVector3<T> > m_vNormals;

  T m_dEps;

  Sphere<T> m_Sphere;
  CBoundingVolumeTree3<AABB3<T>,T,CTraits,CSubdivisionCreator> *m_pBVH;  

private:
  void Traverse(CBoundingVolumeNode3<AABB3<T>,T,CTraits> *pNode);

  std::vector< CTriangle3<T> > m_vTriangles;

  std::list<CBoundingVolumeNode3<AABB3<T>,T,CTraits> *> leaves;

  double distchecks;
  double adding;
  int    ndistchecks;

};

}
#endif
