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
#include "smoother.h"

namespace i3d {

template<class T>
Smoother<T>::Smoother(UnstructuredGrid<T, DTraits> *grid, int iterations) : grid_(grid), iterations_(iterations)
{
  
}

template<class T>
T weightCalculation(T d) {

    T daux;

    if(d < T(3.0))  {
      daux = 2.0 + 1.0 * (3.0 - d);
    } else {
      daux = 2.0 - 0.2 * (d - 3.0);
    }

    return std::max(daux, 0.8);

}

template<class T>
void Smoother<T>::smooth()
{

  T scaleFactor = 5.0 * 6.0 * (6.0/360.0);

  Vector3<T> *coordsNew;
  coordsNew = new Vector3<T>[grid_->nvt_];

  std::vector<T> volWeight(grid_->nvt_, 0);

  int* n = new int[grid_->nvt_];
  std::memset(n, 0, grid_->nvt_*sizeof(int));

  // Calculate the number of incident nodes 
  // as the weight
  for (int i = 0; i<grid_->nmt_; i++)
  {
    int ia = grid_->verticesAtEdge_[i].edgeVertexIndices_[0];
    int ib = grid_->verticesAtEdge_[i].edgeVertexIndices_[1];
    n[ia]++;
    n[ib]++;
  }

  for(auto ive(0); ive < grid_->nel_; ++ive) {
      Hexa &hex = grid_->hexas_[ive];

      Vector3<T> mid = Vector3<T>(0,0,0);

      for(auto vidx(0); vidx < 8; ++vidx) {
          int idx = hex.hexaVertexIndices_[vidx];
          volWeight[ive] += 1.0/(T(n[ive])) *  grid_->elemVol_[ive]; 
      }

  }

  // Calculate the final f weight
  for(auto ivt(0); ivt < grid_->nvt_; ++ivt) {

    T d1 = scaleFactor * grid_->m_myTraits[ivt].distance;
    
    T f = weightCalculation(d1);
    f = std::pow(f, 2.3);
    vertexWeight_.push_back(f);

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
void Smoother<T>::smoothingKernel(Vector3<T> *coords, int* weights, int level)
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
//    if (grid_->verticesAtBoundary_[i])
//      continue;

    //T w = T(grid_->m_VertexVertex[i].m_iNeighbors);
    T w = T(weights[i]);
    grid_->vertexCoords_[i] =
      T(1.0 - alpha) * grid_->vertexCoords_[i] + ((alpha / w) * coords[i]);

  }

}

template<class T>
void Smoother<T>::smoothMultiLevel()
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
void Smoother<T>::prolongate(int level)
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
template class Smoother<Real>;

//----------------------------------------------------------------------------

}

