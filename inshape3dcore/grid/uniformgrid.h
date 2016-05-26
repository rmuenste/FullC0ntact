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
#ifndef UNIFORMGRID_H
#define UNIFORMGRID_H

//===================================================
//                     INCLUDES
//===================================================
#include <list>
#include <aabb3.h>
#include <sphere.h>
#include <fcdefines.hpp>
#include <cuda_runtime_api.h>

namespace i3d {

  class RigidBody;

  class ElementCell
  {
    public:

      ElementCell(){};

      ~ElementCell(){};

      void Insert(int iel){m_lElements.push_back(iel);};

      std::list<int> m_lElements;

  };

  template<typename T>
    struct BasicTraits {

      BasicTraits() {};
      ~BasicTraits() {};

      void init() {};

      void init(const AABB3<T> &aabb, int cells) {};

    };

  template<typename T>
    struct VertexTraits {

      VertexTraits() {};
      ~VertexTraits() {};

      Vector3<T> *vertexCoords_;

      int *fbm_;
      int *fbmVertices_;


      T cellSize_;

      int cells_[3];

      void init() {};

      void init(const AABB3<T> &aabb, int cells)
      {

        //32x32x32
        vertexCoords_ = nullptr;

        T size = T(cells);

        cellSize_ = (2.0*aabb.extents_[0])/size;

        cells_[0] = (2.0*aabb.extents_[0])/cellSize_;
        cells_[1] = (2.0*aabb.extents_[1])/cellSize_;
        cells_[2] = (2.0*aabb.extents_[2])/cellSize_;

        int vx = cells_[0]+1;

        int vxy=vx*vx;

        int vxyz = vxy*vx;

        vertexCoords_ = new Vector3<T>[vxyz];
        fbm_ = new int[cells_[0]*cells_[1]*cells_[2]];
        fbmVertices_ = new int[vxyz];
        for(int i=0; i < cells_[0]*cells_[1]*cells_[2]; ++i)
        {
          fbm_[i] = 0; 
        }

        //generate the vertex coordinates
        for(int k=0;k<vx;k++)
        {
          for(int j=0;j<vx;j++)
          {
            for(int i=0;i<vx;i++)
            {
              vertexCoords_[k*vxy+j*vx+i].x=aabb.vertices_[0].x+i*cellSize_;
              vertexCoords_[k*vxy+j*vx+i].y=aabb.vertices_[0].y+j*cellSize_;
              vertexCoords_[k*vxy+j*vx+i].z=aabb.vertices_[0].z+k*cellSize_;
              fbmVertices_[k*vxy+j*vx+i]=0;
            }
          }
        }
      };
    };


  /**
   * @brief The class implements a uniform grid data structure
   * 
   */
  template<class T, class CellType, class Traits = BasicTraits<T>, int memory=cpu>
    class UniformGrid
    {
      public:

        Traits traits_;

        // boundarybox
        AABB3<T> m_bxBox;

        // dimension
        int m_iDimension[3];

        // cell size
        T m_dCellSize;

        CellType *m_pCells;

        int m_iTotalEntries;

        UniformGrid();

        /**
         * @brief Construct a HUniformGrid from a boundingBox
         *
         * Construct a HUniformGrid from a boundingBox  
         *  
         */  
        UniformGrid(const AABB3<T> &boundingBox, const AABB3<T> &element);  

        ~UniformGrid();

        /**
         * @brief Inits a new grid from a certain element size
         *  
         *  Inits a new grid from a certain element size
         * 
         */  
        void initGrid(const AABB3<T> &boundingBox, const AABB3<T> &element);

        /**
         * @brief Inits a new grid level with a given cellSize 
         *  
         *  Inits a new grid level with a given cellSize 
         * 
         */  
        void initGrid(const AABB3<T> &boundingBox, T cellSize);

        /**
         * @brief Checks which cells of the grid are intersected by the body's bounding box 
         *
         * Checks which cells of the grid are intersected by the body's bounding box. As 
         * a side effect these cells are stored in the body's cell list. 
         * 
         */  
        void query(RigidBody *body);

        /**
         * @brief Checks in which cell a point is located 
         * 
         * The function checks in which cell the query point is located. Cells 
         * of the uniform grid store objects based on their center point and bounding sphere 
         * radius only because of this the elemlist returned contains the objects contained in the 
         * cell that contains the point AND the objects from neighboring cells that could potentially 
         * intersect the cell where the query point is located. 
         * 
         */  
        void pointQuery(const Vector3<T> &q, std::list<int> &elemlist);

        /**
         * @brief Resets the grid, so it can be rebuild with different parameters
         *
         * Resets the grid, so it can be rebuild with different parameters  
         *
         */    
        void reset();

        /**
         * @brief Inserts an element with a certain integer id into the grid 
         * 
         * The element gets inserted into the grid depending on the coordinate 
         * of its center point. The inserted key is the element's id.
         * 
         */  
        void insert(int ielementID, const Vector3<T> &center);

        /**
         * @brief Removes an element from the grid
         * 
         * Removes an element from the grid
         * 
         */  
        void remove();

        inline int getNumEntries(){return m_iTotalEntries;};


    };

  template<class T, class CellType, int memory>
    class UniformGrid<T,CellType,VertexTraits<T>,memory>
    {
      public:

        // boundarybox
        AABB3<T> m_bxBox;

        // dimension
        int m_iDimension[3];

        // cell size
        T m_dCellSize;

        CellType *m_pCells;

        int m_iTotalEntries;

        VertexTraits<T> traits_;

        UniformGrid() {};

        UniformGrid(const AABB3<T> &boundingBox, const AABB3<T> &element);  

        UniformGrid(const UniformGrid<Real, CellType, VertexTraits<Real>, cpu> *copy)
        {

          Vector3<T> vmin, vmax;
          vmin.x = (T)copy->m_bxBox.vertices_[0].x;
          vmin.y = (T)copy->m_bxBox.vertices_[0].y;
          vmin.z = (T)copy->m_bxBox.vertices_[0].z;

          vmax.x = (T)copy->m_bxBox.vertices_[1].x;
          vmax.y = (T)copy->m_bxBox.vertices_[1].y;
          vmax.z = (T)copy->m_bxBox.vertices_[1].z;

          m_bxBox.init(vmin, vmax);

          m_iDimension[0] = copy->m_iDimension[0];
          m_iDimension[1] = copy->m_iDimension[1];
          m_iDimension[2] = copy->m_iDimension[2];

          m_dCellSize = copy->m_dCellSize;

          m_pCells = new ElementCell[m_iDimension[0]*m_iDimension[1]*m_iDimension[2]];

          traits_.init(m_bxBox, m_iDimension[0]);

        } 

        ~UniformGrid(){};

        void initGrid(const AABB3<T> &boundingBox, const AABB3<T> &element)
        {


        }

        void initGrid(const AABB3<Real> &boundingBox, Real cellSize)
        {

          m_dCellSize = cellSize;
          m_bxBox = boundingBox;

          int x = (2.0*m_bxBox.extents_[0])/m_dCellSize;
          int y = (2.0*m_bxBox.extents_[1])/m_dCellSize;
          int z = (2.0*m_bxBox.extents_[2])/m_dCellSize;

          // pass a bounding box that is m_dCellSize bigger in each dimension
          m_pCells = new ElementCell[x*y*z];

          traits_.init(m_bxBox, x);

          // printf("cell size: %f\n",m_dCellSize);
          // printf("domain extends : %f %f %f\n",boundingBox.extents_[0],boundingBox.extents_[1],boundingBox.extents_[2]);
          // printf("element extends : %f %f %f\n",element.extents_[0],element.extents_[1],element.extents_[2]);      
          // printf("Dimensions : %d %d %d\n",x,y,z);      
          // printf("Number of cells in uniform grid : %d\n",x*y*z);    

          m_iDimension[0] = x;
          m_iDimension[1] = y;
          m_iDimension[2] = z;

          m_iTotalEntries=0;

        }

        void query(RigidBody *body)
        {
        }

        void pointQuery(const Vector3<T> &q, std::list<int> &elemlist)
        {
        }

        void vertexIndices(int icellx,int icelly, int icellz, int indices[8])
        {

          int dim = m_iDimension[0]+1; 
          int dim2 = dim * dim; 
          int baseIndex=icellz*dim2+icelly*dim+icellx; 

          indices[0]=baseIndex;         //xmin,ymin,zmin
          indices[1]=baseIndex+1;       //xmax,ymin,zmin

          indices[2]=baseIndex+dim+1; //xmax,ymax,zmin
          indices[3]=baseIndex+dim;   //xmin,ymax,zmin


          indices[4]=baseIndex+dim2;  
          indices[5]=baseIndex+dim2+1;  

          indices[6]=baseIndex+dim+dim2+1;  
          indices[7]=baseIndex+dim+dim2;

        }

        void query(const Sphere<T> &s)
        {

          //compute max overlap at level
          //the max overlap at a level is the maximum distance, 
          //that an object in the neighbouring cell can penetrate
          //into the cell under consideration
          Vector3<T> v = s.center_;

          Vector3<T> origin(m_bxBox.center_.x-m_bxBox.extents_[0],
              m_bxBox.center_.y-m_bxBox.extents_[1],
              m_bxBox.center_.z-m_bxBox.extents_[2]);

          T delta = s.radius_;  
          T invCellSize = 1.0/m_dCellSize;  

          //      int x = (int)std::floor((v.x-origin.x) * invCellSize);
          //      int y = (int)std::floor((v.y-origin.y) * invCellSize);
          //      int z = (int)std::floor((v.z-origin.z) * invCellSize);

          int x0=std::min<int>(int(std::max<T>((v.x-delta-origin.x) * invCellSize,0.0)),m_iDimension[0]-1);   
          int y0=std::min<int>(int(std::max<T>((v.y-delta-origin.y) * invCellSize,0.0)),m_iDimension[1]-1);
          int z0=std::min<int>(int(std::max<T>((v.z-delta-origin.z) * invCellSize,0.0)),m_iDimension[2]-1);

          int x1=std::min<int>(int(std::max<T>((v.x+delta-origin.x) * invCellSize,0.0)),m_iDimension[0]-1);   
          int y1=std::min<int>(int(std::max<T>((v.y+delta-origin.y) * invCellSize,0.0)),m_iDimension[1]-1);   
          int z1=std::min<int>(int(std::max<T>((v.z+delta-origin.z) * invCellSize,0.0)),m_iDimension[2]-1);   

          //loop over the overlapped cells
          for(int x=x0;x<=x1;x++)
            for(int y=y0;y<=y1;y++)
              for(int z=z0;z<=z1;z++)
              {
                int index = z*m_iDimension[1]*m_iDimension[0]+y*m_iDimension[0]+x;
                traits_.fbm_[index]=1;
              }//for z  
        }

        void processVertex(const Sphere<T> &s, int cellIndex, int indices[8])
        {
          for(int i=0; i < 8; ++i)
          {
            if(s.isPointInside(traits_.vertexCoords_[indices[i]]))
            {
              traits_.fbmVertices_[indices[i]]=1;
            } 
          }
        } 

        void queryVertex(const Sphere<T> &s)
        {

          //compute max overlap at level
          //the max overlap at a level is the maximum distance, 
          //that an object in the neighbouring cell can penetrate
          //into the cell under consideration
          Vector3<T> v = s.center_;

          Vector3<T> origin(m_bxBox.center_.x-m_bxBox.extents_[0],
              m_bxBox.center_.y-m_bxBox.extents_[1],
              m_bxBox.center_.z-m_bxBox.extents_[2]);

          T delta = s.radius_;  
          T invCellSize = 1.0/m_dCellSize;  

          int x0=std::min<int>(int(std::max<T>((v.x-delta-origin.x) * invCellSize,0.0)),m_iDimension[0]-1);   
          int y0=std::min<int>(int(std::max<T>((v.y-delta-origin.y) * invCellSize,0.0)),m_iDimension[1]-1);
          int z0=std::min<int>(int(std::max<T>((v.z-delta-origin.z) * invCellSize,0.0)),m_iDimension[2]-1);

          int x1=std::min<int>(int(std::max<T>((v.x+delta-origin.x) * invCellSize,0.0)),m_iDimension[0]-1);   
          int y1=std::min<int>(int(std::max<T>((v.y+delta-origin.y) * invCellSize,0.0)),m_iDimension[1]-1);   
          int z1=std::min<int>(int(std::max<T>((v.z+delta-origin.z) * invCellSize,0.0)),m_iDimension[2]-1);   

          //loop over the overlapped cells
          for(int x=x0;x<=x1;x++)
            for(int y=y0;y<=y1;y++)
              for(int z=z0;z<=z1;z++)
              {
                int indices[8];
                vertexIndices(x,y,z, indices);
                int index = z*m_iDimension[1]*m_iDimension[0]+y*m_iDimension[0]+x;
                processVertex(s,index, indices);
              }//for z  
        }

        void reset()
        {
        }

        void insert(int ielementID, const Vector3<T> &center)
        {
        };

        void remove()
        {
        };

        inline int getNumEntries(){return m_iTotalEntries;};

        void outputInfo()
        {
          printf("Dimension: %i %i %i\n",m_iDimension[0],m_iDimension[1],m_iDimension[2]);
          int vx = m_iDimension[0]+1;
          printf("vertices : %i %i %i\n",vx, vx, vx);
        }


    };

  template<typename T>
    class SimulationParameters
    {

      public:

        T spring_;
        T damping_;
        T shear_;
        T attraction_;
        T boundaryDamping_;

        T globalDamping_;
        T particleRadius_;
        T timeStep_;

        Vector3<T> origin_; 
        Vector3<T> cellSize_; 
        Vector3<T> gravity_; 

        unsigned int gridx_, gridy_, gridz_;
        unsigned int numParticles_;

    };

  template<typename T, int memory>
    class HashGrid
    {
      public:

        unsigned int size_;
        unsigned int numCells_;
        unsigned int *hashEntries_;
        unsigned int *particleIndices_;

        unsigned int *cellStart_;
        unsigned int *cellEnd_;

        unsigned int gridx_;
        unsigned int gridy_;
        unsigned int gridz_;

        Vector3<T> cellSize_;
        Vector3<T> origin_;


        HashGrid() : size_(0), hashEntries_(nullptr), particleIndices_(nullptr)
        {
        };

        ~HashGrid()
        {
        }

        void clean()
        {
          cudaFree(hashEntries_);
          cudaFree(particleIndices_);
          cudaFree(cellEnd_);
          cudaFree(cellStart_);
          cudaDeviceSynchronize();
        };

        void sortGrid(unsigned int *dev_indices)
        {
        }

        void initGrid(unsigned int size, unsigned int *& dev_hash, unsigned int *& dev_indices)
        {

        }

    };

  template<typename T, int memory>
    class ParticleWorld
    {
      public:

        SimulationParameters<T> *params_;
        SimulationParameters<T> *dev_params_;

        T *pos_;
        T *vel_;

        T *sortedPos_;
        T *sortedVel_;

        int size_;

        ParticleWorld() : size_(0), pos_(nullptr),
        vel_(nullptr), sortedPos_(nullptr), sortedVel_(nullptr)
        {
        };

        ~ParticleWorld()
        {
        }

        void update(T dt)
        {
        };

        void setParticles()
        {
        };

        void clean()
        {
          cudaFree(pos_);
          cudaFree(vel_);
          cudaFree(sortedPos_);
          cudaFree(sortedVel_);
          cudaFree(dev_params_);
          cudaDeviceSynchronize();
        };

    };

}

#endif
