#ifndef PARTICLEWORLD_HPP_091XQZT6
#define PARTICLEWORLD_HPP_091XQZT6

#include <uniformgrid.h>
#include <managed.hpp>

namespace i3d
{

  template<typename T>
    class ParticleWorld<T,unified> : public Managed
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

        ParticleWorld(int size) : pos_(nullptr), vel_(nullptr),
                                  sortedPos_(nullptr), sortedVel_(nullptr), size_(size), 
                                  params_(nullptr)
        {
          cudaCheck(cudaMallocManaged((void**)&(pos_), size_* 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(vel_), size_ * 4 * sizeof(T)));

          cudaCheck(cudaMallocManaged((void**)&(sortedPos_), size_ * 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(sortedVel_), size_ * 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(params_), sizeof(SimulationParameters<T>)));
        };

        ~ParticleWorld()
        {
          cudaCheck(cudaFree((pos_)));
          cudaCheck(cudaFree((vel_)));

          cudaCheck(cudaFree((sortedPos_)));
          cudaCheck(cudaFree((sortedVel_)));
          cudaCheck(cudaFree((params_)));
        }

        void init(int size)
        {
          cudaCheck(cudaMallocManaged((void**)&(pos_), size_* 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(vel_), size_ * 4 * sizeof(T)));

          cudaCheck(cudaMallocManaged((void**)&(sortedPos_), size_ * 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(sortedVel_), size_ * 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(params_), sizeof(SimulationParameters<T>)));
        }

        void update(T dt)
        {

        };

        void setParticles()
        {

        };

    };
  
} /* i3d */ 

#endif /* end of include guard: PARTICLEWORLD_HPP_091XQZT6 */
