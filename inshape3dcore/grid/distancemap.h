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
#include <uniformgrid.h>

namespace i3d {

  /**
   * @brief The class implements a signed distance map based on a uniform grid
   * 
   */
  template<typename T, int memory=cpu>
    class DistanceMap
    {

      public:

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

        DistanceMap() : vertexCoords_(nullptr), normals_(nullptr), 
                        contactPoints_(nullptr), distance_(nullptr), stateFBM_(nullptr)

        {
          cells_[0] = 0;
          cells_[1] = 0;
          cells_[2] = 0;
          dim_[0]   = 0;
          dim_[1]   = 0;
          cellSize_ = T(0);
        };

        DistanceMap(const DistanceMap &copy ){

        };

        DistanceMap(DistanceMap<Real,cpu> *copy);

        DistanceMap(const AABB3<T> &aabb);  

        DistanceMap(const AABB3<T> &aabb, int cells);

        DistanceMap(const AABB3<T> &aabb, int cells[]);

        ~DistanceMap();

        void convertToUnstructuredGrid(UnstructuredGrid<T, DTraits>& ugrid);  

        unsigned getVertexCount() const {
          unsigned size = dim_[0] * dim_[1]; 
          return size;
        }

        void outputInfo()
        {
          std::cout<< "Size: " << dim_[1]*dim_[0] << std::endl;
        }

        //ClosestPoint to vertex -> easily compute normal
        T trilinearInterpolateDistance(const Vector3<T> &vQuery, int indices[8]);

        Vector3<T> trilinearInterpolateCP(const Vector3<T> &vQuery, int indices[8]);

        //VertexTraits
        //map cell to vertex(cell index[ci],ci+1,ci+vx,ci+vx+1,ci+vxy,ci+vxy+1,ci+vxy+vx,ci+vxy+vx+1)
        void vertexIndices(int icellx,int icelly, int icellz, int indices[8]);

        //QueryDistanceMap
        std::pair<T, Vector3<T> > queryMap(const Vector3<T> &vQuery);

        int queryInside(const Vector3<T> &vQuery)
        {

          if(!boundingBox_.isPointInside(vQuery))
          {
            return 0;
          }

          //calculate the cell indices
          T invCellSize = 1.0/cellSize_;

          //calculate the cell indices
          int x = (int)(std::abs(vQuery.x-boundingBox_.vertices_[0].x) * invCellSize);
          int y = (int)(std::abs(vQuery.y-boundingBox_.vertices_[0].y) * invCellSize);
          int z = (int)(std::abs(vQuery.z-boundingBox_.vertices_[0].z) * invCellSize);  

          //vertex indices
          int indices[8];

          //look up distance for the cell
          vertexIndices(x,y,z,indices);

          T first  = trilinearInterpolateDistance(vQuery,indices);      
          //printf(" vertexCoords = %f %f %f = %f\n", vQuery.x, vQuery.y, first);
                
          if(first <= T(0.0))
            return 1;
          else
            return 0;

        };

    };

}
#endif
