#ifndef UNIFORMGRID_CUH_NPGCYKTM
#define UNIFORMGRID_CUH_NPGCYKTM

#include <uniformgrid.h>

namespace i3d {

  template<typename T>
    class ParticleWorld<T,gpu>
    {
      public:

        T *pos_;
        T *vel_;

        T *sortedPos_;
        T *sortedVel_;

        int size_;

        SimulationParameters<T> *params_;

        ParticleWorld() : pos_(nullptr), vel_(nullptr),
                          sortedPos_(nullptr), sortedVel_(nullptr), size_(0), 
                          params_(nullptr)
        {

        };

        void update(T dt)
        {

        };

        void setParticles()
        {

        };

        void deepCopy(ParticleWorld<T,cpu> &pw, SimulationParameters<T>*& dev_params)
        {
          unsigned memorySize = pw.size_ * 4 * sizeof(T);
          cudaCheck(cudaMalloc((void**)&pw.pos_, memorySize));
          cudaCheck(cudaMemcpy(&pos_, &pw.pos_, sizeof(T*), cudaMemcpyHostToDevice)); 

          cudaCheck(cudaMalloc((void**)&pw.vel_, memorySize));
          cudaCheck(cudaMemcpy(&vel_, &pw.vel_, sizeof(T*), cudaMemcpyHostToDevice)); 

          cudaCheck(cudaMalloc((void**)&pw.sortedPos_, memorySize));
          cudaCheck(cudaMemcpy(&sortedPos_, &pw.sortedPos_, sizeof(T*), cudaMemcpyHostToDevice)); 

          cudaCheck(cudaMalloc((void**)&pw.sortedVel_, memorySize));
          cudaCheck(cudaMemcpy(&sortedVel_, &pw.sortedVel_, sizeof(T*), cudaMemcpyHostToDevice)); 

          cudaCheck(cudaMalloc((void**)&dev_params, sizeof(SimulationParameters<T>)));
          cudaCheck(cudaMemcpy(dev_params, pw.params_, sizeof(SimulationParameters<T>), cudaMemcpyHostToDevice)); 
          cudaCheck(cudaMemcpy(&params_, &dev_params, sizeof(SimulationParameters<T>*), cudaMemcpyHostToDevice)); 

        };

    };

  template<typename T>
    class HashGrid<T,gpu>
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

        HashGrid() : size_(0), hashEntries_(nullptr), particleIndices_(nullptr),
                     cellStart_(nullptr), cellEnd_(nullptr)
        {

        };

        void sortGrid(unsigned int *dev_indices)
        {
          thrust::sort(thrust::device_ptr<unsigned int>(dev_indices),
                       thrust::device_ptr<unsigned int>(dev_indices)+10);
        }

        void sortParticles(unsigned int size, unsigned int *dev_hash, unsigned int *dev_indices)
        {
          thrust::sort_by_key(thrust::device_ptr<unsigned int>(dev_hash),
                              thrust::device_ptr<unsigned int>(dev_hash+size),
                              thrust::device_ptr<unsigned int>(dev_indices));
        }

        void deepCopy(HashGrid<T, cpu> &hg)
        {
          cudaCheck(cudaMalloc((void**)&hg.hashEntries_, hg.size_ * sizeof(unsigned int)));
          cudaCheck(cudaMemcpy(&hashEntries_, &hg.hashEntries_, sizeof(unsigned int*), cudaMemcpyHostToDevice)); 

          cudaCheck(cudaMalloc((void**)&hg.particleIndices_, hg.size_ * sizeof(unsigned int)));
          cudaCheck(cudaMemcpy(&particleIndices_, &hg.particleIndices_, sizeof(unsigned int*), cudaMemcpyHostToDevice)); 
          
          cudaCheck(cudaMalloc((void**)&hg.cellStart_, hg.numCells_ * sizeof(unsigned int)));
          cudaCheck(cudaMemcpy(&cellStart_, &hg.cellStart_, sizeof(unsigned int*), cudaMemcpyHostToDevice)); 

          cudaCheck(cudaMalloc((void**)&hg.cellEnd_, hg.numCells_ * sizeof(unsigned int)));
          cudaCheck(cudaMemcpy(&cellEnd_, &hg.cellEnd_, sizeof(unsigned int*), cudaMemcpyHostToDevice)); 
        }

        __device__ 
        void setParticleHash(unsigned int hash, unsigned int index)
        {
          hashEntries_[index] = hash;
          particleIndices_[index] = index;
        }

        __device__ 
        void outputInfo()
        {
          for(int i(0); i < size_; ++i)
          {
            printf("particleIndices_[%i]=%i\n",i,particleIndices_[i]);  
          }
        }

        __device__ int3 getGridIndex(float3 p)
        {
            int3 gridIndex;
            gridIndex.x = floor((p.x - origin_.x) / cellSize_.x);
            gridIndex.y = floor((p.y - origin_.y) / cellSize_.y);
            gridIndex.z = floor((p.z - origin_.z) / cellSize_.z);
            return gridIndex;
        }

        __device__ unsigned int hash(int3 gridIndex)
        {
          gridIndex.x = gridIndex.x & (gridx_-1);
          gridIndex.y = gridIndex.y & (gridy_-1);
          gridIndex.z = gridIndex.z & (gridz_-1);        
          return __umul24(__umul24(gridIndex.z, gridy_), gridx_) +
                 __umul24(gridIndex.y, gridx_) + gridIndex.x;
        }

    };

  template<class T, class CellType>
    class UniformGrid<T,CellType,VertexTraits<T>,gpu>
    {
      public:

        // boundarybox
        AABB3<T> boundingBox_;

        // dimension
        int dim_[3];

        // cell size
        T cellSize_;

        CellType *cells_;

        int entries_;

        VertexTraits<T> traits_;

        UniformGrid() {};

        UniformGrid(const AABB3<T> &boundingBox, const AABB3<T> &element);  

        ~UniformGrid(){};

        void initGrid(const AABB3<T> &boundingBox, const AABB3<T> &element)
        {

        }

        void initGrid(const AABB3<Real> &boundingBox, Real cellSize)
        {

          cellSize_ = cellSize;
          boundingBox_ = boundingBox;

          int x = (2.0*boundingBox_.extents_[0])/cellSize_;
          int y = (2.0*boundingBox_.extents_[1])/cellSize_;
          int z = (2.0*boundingBox_.extents_[2])/cellSize_;

          // pass a bounding box that is cellSize_ bigger in each dimension
          cells_ = new ElementCell[x*y*z];

          traits_.init(boundingBox_, x);

          dim_[0] = x;
          dim_[1] = y;
          dim_[2] = z;

          entries_=0;

        }

        void query(RigidBody *body)
        {
        }

        void pointQuery(const Vector3<T> &q, std::list<int> &elemlist)
        {
        }

        __device__  
          void vertexIndices(int icellx,int icelly, int icellz, int indices[8])
          {

            int dim = dim_[0]+1; 
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

          Vector3<T> origin(boundingBox_.center_.x-boundingBox_.extents_[0],
              boundingBox_.center_.y-boundingBox_.extents_[1],
              boundingBox_.center_.z-boundingBox_.extents_[2]);

          T delta = s.radius_;  
          T invCellSize = 1.0/cellSize_;  

          int x0=std::min<int>(int(std::max<T>((v.x-delta-origin.x) * invCellSize,0.0)),dim_[0]-1);   
          int y0=std::min<int>(int(std::max<T>((v.y-delta-origin.y) * invCellSize,0.0)),dim_[1]-1);
          int z0=std::min<int>(int(std::max<T>((v.z-delta-origin.z) * invCellSize,0.0)),dim_[2]-1);

          int x1=std::min<int>(int(std::max<T>((v.x+delta-origin.x) * invCellSize,0.0)),dim_[0]-1);   
          int y1=std::min<int>(int(std::max<T>((v.y+delta-origin.y) * invCellSize,0.0)),dim_[1]-1);   
          int z1=std::min<int>(int(std::max<T>((v.z+delta-origin.z) * invCellSize,0.0)),dim_[2]-1);   

          //loop over the overlapped cells
          for(int x=x0;x<=x1;x++)
            for(int y=y0;y<=y1;y++)
              for(int z=z0;z<=z1;z++)
              {
                int index = z*dim_[1]*dim_[0]+y*dim_[0]+x;
                traits_.fbm_[index]=1;
              }//for z  
        }

        __device__  
          void processVertex(const Sphere<T> &s, int cellIndex, int indices[8])
          {
            for(int i=0; i < 8; ++i)
            {

              if((traits_.vertexCoords_[indices[i]] - s.center_).norm2() <= s.radius_*s.radius_)
              {
                traits_.fbmVertices_[indices[i]]=1;
              } 
              //printf("vertex: %i %i %i\n",indices[i],i,traits_.fbmVertices_[indices[i]]);
            }
          } 

        __device__  
          void queryVertex(const Sphere<T> &s)
          {

            //compute max overlap at level
            //the max overlap at a level is the maximum distance, 
            //that an object in the neighbouring cell can penetrate
            //into the cell under consideration
            Vector3<T> v = s.center_;

            Vector3<T> origin(boundingBox_.center_.x-boundingBox_.extents_[0],
                boundingBox_.center_.y-boundingBox_.extents_[1],
                boundingBox_.center_.z-boundingBox_.extents_[2]);

            T delta = s.radius_;  
            T invCellSize = 1.0/cellSize_;  

            int x0=min(int(fmaxf((v.x-delta-origin.x) * invCellSize,0.0)),dim_[0]-1);   
            int y0=min(int(fmaxf((v.y-delta-origin.y) * invCellSize,0.0)),dim_[1]-1);
            int z0=min(int(fmaxf((v.z-delta-origin.z) * invCellSize,0.0)),dim_[2]-1);

            int x1=min(int(fmaxf((v.x+delta-origin.x) * invCellSize,0.0)),dim_[0]-1);   
            int y1=min(int(fmaxf((v.y+delta-origin.y) * invCellSize,0.0)),dim_[1]-1);   
            int z1=min(int(fmaxf((v.z+delta-origin.z) * invCellSize,0.0)),dim_[2]-1);   

            //loop over the overlapped cells
            for(int x=x0;x<=x1;x++)
              for(int y=y0;y<=y1;y++)
                for(int z=z0;z<=z1;z++)
                {
                  int indices[8];
                  vertexIndices(x,y,z, indices);
                  int index = z*dim_[1]*dim_[0]+y*dim_[0]+x;
                  processVertex(s,index, indices);
                }//for z  
          }

        void deepCopy(const UniformGrid<T,CellType,VertexTraits<T>,cpu> &grid)
        {

          CellType *dev_cells;

          int size = grid.m_iDimension[0]*grid.m_iDimension[1]*grid.m_iDimension[2]; 

          cudaCheck(cudaMalloc((void**)&dev_cells, size * sizeof(CellType)));

          Vector3<T> *dev_vertexCoords;
          int *dev_fbm;
          int *dev_fbmVertices;

          int vx = grid.traits_.cells_[0]+1;
          int vy = grid.traits_.cells_[1]+1;
          int vz = grid.traits_.cells_[2]+1;

          size = vx * vy * vz;

          cudaCheck(cudaMalloc((void**)&dev_vertexCoords, size * sizeof(Vector3<T>)));

          cudaCheck(cudaMemcpy(dev_vertexCoords, grid.traits_.vertexCoords_, size * sizeof(Vector3<T>), cudaMemcpyHostToDevice));

          cudaCheck(cudaMemcpy(&traits_.vertexCoords_ , &dev_vertexCoords, sizeof(Vector3<T>*), cudaMemcpyHostToDevice));

          cudaCheck(cudaMalloc((void**)&dev_fbm, size * sizeof(int)));

          cudaCheck(cudaMemcpy(dev_fbm, grid.traits_.fbm_, size * sizeof(int), cudaMemcpyHostToDevice));

          cudaCheck(cudaMemcpy(&traits_.fbm_ , &dev_fbm, sizeof(int*), cudaMemcpyHostToDevice));

          cudaCheck(cudaMalloc((void**)&dev_fbmVertices, size * sizeof(int)));

          cudaCheck(cudaMemcpy(dev_fbmVertices, grid.traits_.fbmVertices_, size * sizeof(int), cudaMemcpyHostToDevice));

          cudaCheck(cudaMemcpy(&traits_.fbmVertices_ , &dev_fbmVertices, sizeof(int*), cudaMemcpyHostToDevice));
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

        inline int getNumEntries(){return entries_;};

        void outputInfo()
        {
          printf("Dimension: %i %i %i\n",dim_[0],dim_[1],dim_[2]);
        }


    };

}

#endif /* end of include guard: UNIFORMGRID_CUH_NPGCYKTM */
