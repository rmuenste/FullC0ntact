
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
#include <laplace_alpha.hpp> 

namespace i3d {

template<class T>
LaplaceAlpha<T>::LaplaceAlpha(UnstructuredGrid<T, DTraits> *grid, MeshObject<T, cgalKernel> *geom, int iterations) : grid_(grid), iterations_(iterations), geom_(geom)
{
  
}

template<class T>
void LaplaceAlpha<T>::calculateVolumeWeight() {

  grid_->calcVol();
  for(auto ive(0); ive < grid_->nel_; ++ive) {
      Hexa &hex = grid_->hexas_[ive];

      Vector3<T> mid = Vector3<T>(0,0,0);

      for(auto vidx(0); vidx < 8; ++vidx) {
          int idx = hex.hexaVertexIndices_[vidx];
          volWeight_[idx] += 1.0/(T(n[idx])) *  grid_->elemVol_[ive]; 
      }

  }

}

template<class T>
T LaplaceAlpha<T>::weightCalculation(T d) {

    T daux;

    if(d < T(0.0))  {
        d = std::abs(d) * 1.0;
    }

    if(d < T(3.0))  {
      daux = 2.0 + 1.0 * (3.0 - d);
    } else {
      daux = 2.0 - 0.2 * (d - 3.0);
    }

    return std::max(daux, 0.8);

}

template<class T>
void LaplaceAlpha<T>::calculateVertexWeight() {

  T scaleFactor = 5.0 * 6.0 * (6.0/3.0);

  // Calculate the f weight
  for(auto ivt(0); ivt < grid_->nvt_; ++ivt) {

    T d1 = scaleFactor * grid_->m_myTraits[ivt].distance;
    
    T volumeContribution = std::abs(volWeight_[ivt]);

    T distanceContribution = weightCalculation(d1);

    //std::cout << "weight: " << ivt << " " << f << std::endl;

    distanceContribution = std::pow(distanceContribution, 2.3);

    vertexWeight_[ivt] = distanceContribution * volumeContribution;

    //vertexWeight_[ivt] = std::max(0.2, std::pow(grid_->vertexCoords_[ivt].x, 2.0));
    grid_->m_myTraits[ivt].dist2 = grid_->m_myTraits[ivt].distance;
    grid_->m_myTraits[ivt].distance = vertexWeight_[ivt];

  }

}

template<class T>
void LaplaceAlpha<T>::smooth()
{

  Vector3<T> *coordsNew;
  coordsNew = new Vector3<T>[grid_->nvt_];

  for (int i = 0; i<grid_->nvt_; i++)
  {
    volWeight_.push_back(0);
    n.push_back(0);
    vertexWeight_.push_back(0);
    cumulativeWeight_.push_back(0);
  }

  DistanceGridMesh<Real> distance(grid_, geom_);

//    distance.ComputeDistance();

//  int* n = new int[grid_->nvt_];
//  std::memset(n, 0, grid_->nvt_*sizeof(int));

  // Calculate the number of incident nodes 
  // as the weight
  for (int i = 0; i<grid_->nmt_; i++)
  {
    int ia = grid_->verticesAtEdge_[i].edgeVertexIndices_[0];
    int ib = grid_->verticesAtEdge_[i].edgeVertexIndices_[1];
    n[ia]++;
    n[ib]++;
  }

  for (int j = 0; j < iterations_; j++)
  {
    distance.ComputeDistance();
    calculateVolumeWeight();
    calculateVertexWeight();
    smoothingKernel(coordsNew,(grid_->refinementLevel_));

    for (int i = 0; i < grid_->nvt_; i++) {
      coordsNew[i] = Vector3<T>(0, 0, 0);
      volWeight_[i] = 0;
      vertexWeight_[i] = 0;
      cumulativeWeight_[i] = 0;
    }

  }

  delete[] coordsNew;

}

template<class T>
void LaplaceAlpha<T>::smoothingKernel(Vector3<T> *coords, int level)
{

  T alpha = 0.666;
  for (int i = 0; i < grid_->nmtLev_[level]; i++)
  {
    int a = grid_->verticesAtEdgeLev_[level][i].edgeVertexIndices_[0];
    int b = grid_->verticesAtEdgeLev_[level][i].edgeVertexIndices_[1];

    coords[a] += vertexWeight_[b] * grid_->vertexCoords_[b];
    coords[b] += vertexWeight_[a] * grid_->vertexCoords_[a];
    cumulativeWeight_[a] += vertexWeight_[b]; 
    cumulativeWeight_[b] += vertexWeight_[a];

  }

  for(int i(0); i < grid_->nvt_; ++i) {
    coords[i] /= cumulativeWeight_[i];
    //std::cout << "weight: " << i << " " << cumulativeWeight_[i] << std::endl;
  }

  for (int i = 0; i < grid_->nvtLev_[level]; i++)
  {
    if (grid_->verticesAtBoundary_[i]) {
      moveBoundaryVertex(coords, i); 
    } else {
      T w = T(n[i]);
      grid_->vertexCoords_[i] =
      T(1.0 - alpha) * grid_->vertexCoords_[i] + ((alpha) * coords[i]);
    }
  }
}

template<class T>
void LaplaceAlpha<T>::moveBoundaryVertex(Vector3<T> *coords, unsigned idx) {

  int elemAtVertex = grid_->elementsAtVertexIdx_[idx+1] - grid_->elementsAtVertexIdx_[idx];

  if (elemAtVertex == 1) 
    return;
  else if (elemAtVertex == 2) {
    return;
  } else {
    int start = grid_->elementsAtVertexIdx_[idx];
    int end = grid_->elementsAtVertexIdx_[idx+1];
    std::cout << "incident hexas: " << end - start << std::endl;

    int eidx = grid_->elementsAtVertex_[start];
    Hexa &hex = grid_->hexas_[eidx];

    int cnt = 0;
    // find the boundary face
    for(auto fidx(0); fidx < 6; ++fidx) {
        // we found the boundary face
        if(hex.hexaNeighborIndices_[fidx] == -1) 
          cnt++;
    }

    if(cnt > 1)
      return;

    // find the boundary face
    for(auto fidx(0); fidx < 6; ++fidx) {
    
        // we found the boundary face
        if(hex.hexaNeighborIndices_[fidx] == -1) {

            int iface = hex.hexaFaceIndices_[fidx];
            HexaFace &hface = grid_->verticesAtFace_[iface];

//            int v0 = hface.faceVertexIndices_[0];
//            int v1 = hface.faceVertexIndices_[1];
//            int v2 = hface.faceVertexIndices_[2];
//
//            Vector3<T> va = grid_->vertexCoords_[v1] -grid_->vertexCoords_[v0];   
//            Vector3<T> vb = grid_->vertexCoords_[v2] -grid_->vertexCoords_[v0];   
//            
//            Vector3<T> normal = Vector3<T>::Cross(va, vb);
//            normal.normalize();
//            if(normal * grid_->vertexCoords_[idx] < 0.0) {
//                normal = -normal;
//            }
            
            Vector3<T> normal = grid_->faceNormals_[iface]; 
            T alpha = 0.666; 
            Vector3<T> displacement = coords[idx] - (normal * (coords[idx] * normal));
            std::cout << "x: " << grid_->vertexCoords_[idx];
            std::cout << "normal: " << normal;
            std::cout << "dot: " << normal * grid_->vertexCoords_[idx] << std::endl;
            std::cout << "disp: " << displacement;
            std::cout << "coords[idx]: " << coords[idx];
            std::cout << "[idx]: " << idx;
            std::cout << "--------------------------" << std::endl;
            grid_->vertexCoords_[idx] =
            T(1.0 - alpha) * grid_->vertexCoords_[idx] + ((alpha) * coords[idx]);

            if(normal.x == 1.0) {
              grid_->vertexCoords_[idx].x = grid_->maxVertex_.x;
            } else if(normal.y == 1.0) {
              grid_->vertexCoords_[idx].y = grid_->maxVertex_.y;
            } else if(normal.z == 1.0) {
              grid_->vertexCoords_[idx].z = grid_->maxVertex_.z;
            } else if(normal.x == -1.0) {
              grid_->vertexCoords_[idx].x = grid_->minVertex_.x;
            } else if(normal.y == -1.0) {
              grid_->vertexCoords_[idx].y = grid_->minVertex_.y;
            } else if(normal.z == -1.0) {
              grid_->vertexCoords_[idx].z = grid_->minVertex_.z;
            } else {

            }
            

            break;
        }
    }
       
    return;
  }

  //std::cout << "#Elems at vertex: " << elemAtVertex << std::endl;

}

template<class T>
void LaplaceAlpha<T>::smoothMultiLevel()
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
      smoothingKernel(coordsNew, ilevel);
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
    smoothingKernel(coordsNew, (grid_->refinementLevel_));
    for (int i = 0; i < grid_->nvt_; i++)
      coordsNew[i] = Vector3<T>(0, 0, 0);
  }

  delete[] n;
  delete[] coordsNew;

}

template<class T>
void LaplaceAlpha<T>::prolongate(int level)
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
template class LaplaceAlpha<Real>;

//----------------------------------------------------------------------------

}

