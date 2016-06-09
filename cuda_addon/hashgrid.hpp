#ifndef HASHGRID_HPP_JLCX1VGB
#define HASHGRID_HPP_JLCX1VGB

#include <uniformgrid.h>
#include <managed.hpp>

namespace i3d
{

  template<typename T>
    class HashGrid<T,unified> : public Managed
    {
      public:

        unsigned int size_;
        unsigned int *hashEntries_;
        unsigned int *particleIndices_;

        unsigned int *cellStart_;
        unsigned int *cellEnd_;

        unsigned int gridx_;
        unsigned int gridy_;
        unsigned int gridz_;
        unsigned int numCells_;

        Vector3<T> cellSize_;
        Vector3<T> origin_;

        HashGrid() : size_(0), 
                     hashEntries_(nullptr), particleIndices_(nullptr),
                     cellStart_(nullptr), cellEnd_(nullptr),
                     gridx_(0), gridy_(0), gridz_(0), numCells_(0)
        {

        };

        HashGrid(unsigned size, 
                 unsigned gridx, unsigned gridy, unsigned gridz,
                 const Vector3<T> &cellSize, const Vector3<T> origin) : 
        size_(size), 
        hashEntries_(nullptr), particleIndices_(nullptr),
        cellStart_(nullptr), cellEnd_(nullptr),cellSize_(cellSize), origin_(origin),
        gridx_(gridx), gridy_(gridy), gridz_(gridz), numCells_(gridx*gridy*gridz)
        {

          cudaCheck(cudaMallocManaged((void**)&hashEntries_, size_ * sizeof(unsigned int)));
          cudaCheck(cudaMallocManaged((void**)&particleIndices_, size_ * sizeof(unsigned int)));

          cudaCheck(cudaMallocManaged((void**)&cellStart_, numCells_ * sizeof(unsigned int)));
          cudaCheck(cudaMallocManaged((void**)&cellEnd_, numCells_ * sizeof(unsigned int)));

        };

        ~HashGrid()
        {

          cudaCheck(cudaFree(hashEntries_));
          cudaCheck(cudaFree(particleIndices_));

          cudaCheck(cudaFree(cellStart_));
          cudaCheck(cudaFree(cellEnd_));

        };

        void sortParticles()
        {
          thrust::sort_by_key(thrust::device_ptr<unsigned int>(hashEntries_),
              thrust::device_ptr<unsigned int>(hashEntries_+size_),
              thrust::device_ptr<unsigned int>(particleIndices_));
        }

        __host__ __device__ 
          void setParticleHash(unsigned int hash, unsigned int index)
          {
            hashEntries_[index] = hash;
            particleIndices_[index] = index;
          }

        __host__ __device__ 
          void outputInfo()
          {
            for(int i(0); i < size_; ++i)
            {
              printf("particleIndices_[%i]=%i\n",i,particleIndices_[i]);  
            }
          }

        __host__ __device__ int3 getGridIndex(float3 p)
        {
          int3 gridIndex;
          gridIndex.x = floor((p.x - origin_.x) / cellSize_.x);
          gridIndex.y = floor((p.y - origin_.y) / cellSize_.y);
          gridIndex.z = floor((p.z - origin_.z) / cellSize_.z);
          return gridIndex;
        }

        __host__ __device__ unsigned int hash(int3 gridIndex)
        {
          gridIndex.x = gridIndex.x & (gridx_-1);
          gridIndex.y = gridIndex.y & (gridy_-1);
          gridIndex.z = gridIndex.z & (gridz_-1);        
          return gridIndex.z * gridy_ * gridx_ + gridIndex.y * gridx_ + gridIndex.x;
        }

    };

} /* i3d */ 



#endif /* end of include guard: HASHGRID_HPP_JLCX1VGB */
