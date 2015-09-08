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

enum mem {
  cpu,
  gpu
};

/**
 * @brief The class implements a signed distance map based on a uniform grid
 * 
 */
template<typename T, int memory=cpu>
class DistanceMap
{
public:
  DistanceMap(){};
  
  DistanceMap(const AABB3<T> &aabb);  
  
  DistanceMap(const AABB3<T> &aabb, int cells);

  ~DistanceMap();
  
  void convertToUnstructuredGrid(CUnstrGridr &ugrid);  

  //ClosestPoint to vertex -> easily compute normal
  T trilinearInterpolateDistance(const Vector3<T> &vQuery, int indices[8]);
  
  Vector3<T> trilinearInterpolateCP(const Vector3<T> &vQuery, int indices[8]);
  
  //VertexTraits
  //map cell to vertex(cell index[ci],ci+1,ci+vx,ci+vx+1,ci+vxy,ci+vxy+1,ci+vxy+vx,ci+vxy+vx+1)
  void vertexIndices(int icellx,int icelly, int icellz, int indices[8]);
  
  //QueryDistanceMap
  std::pair<T, Vector3<T> > queryMap(const Vector3<T> &vQuery);
  
  Vector3<T> *vertexCoords_;
  Vector3<T> *normals_;
  Vector3<T> *contactPoints_;      
  
  T *distance_;

  int *stateFBM_;
  
  AABB3<T> boundingBox_;
  
  int cells_[3];
  
  int dim_[2];
  
  // cell size
  T cellSize_;  

};

}
#endif
