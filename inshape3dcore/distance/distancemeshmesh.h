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
#ifndef DISTANCEMESHMESH_H
#define DISTANCEMESHMESH_H



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

namespace i3d {

/**
* @brief Computes the distance between two triangle meshes
* 
*/    
template <typename T>
class CDistanceMeshMesh : public CDistance<T> {

public: 

  CDistanceMeshMesh(); 

  CDistanceMeshMesh(CBoundingVolumeTree3<CAABB3<T>,T,CTraits,CSubdivisionCreator> *pBVH0,
                    CBoundingVolumeTree3<CAABB3<T>,T,CTraits,CSubdivisionCreator> *pBVH1 ) : m_pBVH0(pBVH0), m_pBVH1(pBVH1) {}; 

  ~CDistanceMeshMesh();

  T ComputeDistanceSqr();

  T ComputeDistance();

  T ComputeDistanceEps(T eps);

  std::vector<CVector3<T> > m_vPoint;

  std::vector<CVector3<T> > m_vNormals;

  T m_dEps;

  CBoundingVolumeTree3<CAABB3<T>,T,CTraits,CSubdivisionCreator> *m_pBVH0;
  CBoundingVolumeTree3<CAABB3<T>,T,CTraits,CSubdivisionCreator> *m_pBVH1;

  std::vector< CTriangle3<T> > m_vTriangles;

  std::list<std::pair<CBoundingVolumeNode3<CAABB3<T>,T,CTraits>*,
                      CBoundingVolumeNode3<CAABB3<T>,T,CTraits>* > > pairs;


};

}
#endif
