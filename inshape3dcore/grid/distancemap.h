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
#ifndef DISTANCEMAP_H
#define DISTANCEMAP_H

//===================================================
//                     INCLUDES
//===================================================
#include <list>
#include <aabb3.h>
#include <unstructuredgrid.h> 

namespace i3d {

/**
 * @brief The class implements a signed distance map based on a uniform grid
 * 
 */
template<class T>  
class CDistanceMap
{
public:
  CDistanceMap(){};
  
  CDistanceMap(const CAABB3<T> &aabb);  
  
  ~CDistanceMap();
  
  void ConvertToUnstructuredGrid(CUnstrGridr &ugrid);  
  
  //cellArray
  
  //vertexArray
  
  //ClosestPoint to vertex -> easily compute normal
  T TrilinearInterpolateDistance(const CVector3<T> &vQuery, int indices[8]);
  CVector3<T> TrilinearInterpolateCP(const CVector3<T> &vQuery, int indices[8]);
  
  //VertexTraits
  //map cell to vertex(cell index[ci],ci+1,ci+vx,ci+vx+1,ci+vxy,ci+vxy+1,ci+vxy+vx,ci+vxy+vx+1)
  void VertexIndices(int icellx,int icelly, int icellz, int indices[8]);
  
  //QueryDistanceMap
  std::pair<T, CVector3<T> > Query(const CVector3<T> &vQuery);
  
  CVector3<T> *m_pVertexCoords;
  CVector3<T> *m_pNormals;
  CVector3<T> *m_pContactPoints;      
  
  T *m_dDistance;
  
  CAABB3<T> m_bxBox;
  
  int m_iCells[3];
  int m_iDim[2];
  
  // cell size
  T m_dCellSize;  

};

}
#endif
