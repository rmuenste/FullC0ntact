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
#include <list>
#include <aabb3.h>
#include "laplace.h"

namespace i3d {

template<class T>
Laplace<T>::Laplace(UnstructuredGrid<T, DTraits> *grid, int iterations) : grid_(grid), iterations_(iterations)
{
  
}

template<class T>
void Laplace<T>::smooth()
{

  Vector3<T> *coordsNew;
  coordsNew = new Vector3<T>[grid_->nvt_];

  int* n = new int[grid_->nvt_];
  memset(n, 0, grid_->nvt_*sizeof(int));
  for (int i = 0; i<grid_->nmt_; i++)
  {
    int ia = grid_->verticesAtEdge_[i].edgeVertexIndices_[0];
    int ib = grid_->verticesAtEdge_[i].edgeVertexIndices_[1];
    n[ia]++;
    n[ib]++;
  }


  for (int j = 0; j < iterations_; j++)
  {
    smoothingKernel(coordsNew,n,(grid_->refinementLevel_));
    for (int i = 0; i < grid_->nvt_; i++)
      coordsNew[i] = Vector3<T>(0, 0, 0);
  }

  delete[] n;
  delete[] coordsNew;

}

template<class T>
void Laplace<T>::smoothMultiLevel()
{

  Vector3<T> *coordsNew;
  coordsNew = new Vector3<T>[grid_->nvt_];
  int* n = new int[grid_->nvt_];
  memset(n, 0, grid_->nvt_*sizeof(int));

  for (int ilevel = 0; ilevel <= grid_->refinementLevel_-1; ilevel++)
  {

    for (int i = 0; i<grid_->nmtLev_[ilevel]; i++)
    {
      int ia = grid_->verticesAtEdgeLev_[ilevel][i].edgeVertexIndices_[0];
      int ib = grid_->verticesAtEdgeLev_[ilevel][i].edgeVertexIndices_[1];
      n[ia]++;
      n[ib]++;
    }

    for (int j = 0; j < iterations_; j++)
    {
      smoothingKernel(coordsNew, n, ilevel);
      for (int i = 0; i < grid_->nvtLev_[ilevel]; i++)
        coordsNew[i] = Vector3<T>(0, 0, 0);
    }

    prolongate(ilevel);
    memset(n, 0, grid_->nvt_*sizeof(int));

  }

  for (int i = 0; i<grid_->nmt_; i++)
  {
    int ia = grid_->verticesAtEdge_[i].edgeVertexIndices_[0];
    int ib = grid_->verticesAtEdge_[i].edgeVertexIndices_[1];
    n[ia]++;
    n[ib]++;
  }


  for (int j = 0; j < iterations_; j++)
  {
    smoothingKernel(coordsNew, n, (grid_->refinementLevel_));
    for (int i = 0; i < grid_->nvt_; i++)
      coordsNew[i] = Vector3<T>(0, 0, 0);
  }

  delete[] n;
  delete[] coordsNew;

}

template<class T>
void Laplace<T>::smoothingKernel(Vector3<T> *coords, int* weights, int level)
{

  T alpha = 0.5;
  for (int i = 0; i < grid_->nmtLev_[level]; i++)
  {
    int a = grid_->verticesAtEdgeLev_[level][i].edgeVertexIndices_[0];
    int b = grid_->verticesAtEdgeLev_[level][i].edgeVertexIndices_[1];
    coords[a] += grid_->vertexCoords_[b];
    coords[b] += grid_->vertexCoords_[a];
  }

  for (int i = 0; i < grid_->nvtLev_[level]; i++)
  {
    if (grid_->verticesAtBoundary_[i])
      continue;

    //T w = T(grid_->m_VertexVertex[i].m_iNeighbors);
    T w = T(weights[i]);
    grid_->vertexCoords_[i] =
      T(1.0 - alpha) * grid_->vertexCoords_[i] + ((alpha / w) * coords[i]);

  }

}

template<class T>
void Laplace<T>::prolongate(int level)
{

  int nvt = grid_->nvtLev_[level];
  int nmt = grid_->nmtLev_[level];
  int nat = grid_->natLev_[level];
  int nel = grid_->nelLev_[level];

  for (int i = 0; i < nmt; i++)
  {
    int ivt1 = grid_->verticesAtEdgeLev_[level][i].edgeVertexIndices_[0];
    int ivt2 = grid_->verticesAtEdgeLev_[level][i].edgeVertexIndices_[1];

    Vector3<T> vMid = T(0.5) * (grid_->vertexCoords_[ivt1] + grid_->vertexCoords_[ivt2]);

    grid_->vertexCoords_[nvt + i] = vMid;
  }

  for (int i = 0; i < nat; i++)
  {
    Vector3<T> vMid(0, 0, 0);
    for (int j = 0; j < 4; j++)
    {
      vMid += grid_->vertexCoords_[grid_->verticesAtFaceLev_[level][i].faceVertexIndices_[j]];
    }//end for

    grid_->vertexCoords_[nvt + nmt + i] = vMid*(T)0.25;

  }//end for

  for (int i = 0; i < nel; i++)
  {

    Vector3<T> vMid(0, 0, 0);
    for (int j = 0; j<8; j++)
    {
      vMid += grid_->vertexCoords_[grid_->verticesAtHexaLev_[level][i].hexaVertexIndices_[j]];
    }//end for j

    grid_->vertexCoords_[nvt + nmt + nat + i] = vMid*(T)0.125;

  }//end for i

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class Laplace<Real>;

//----------------------------------------------------------------------------

}

