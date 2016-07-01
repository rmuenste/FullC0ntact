#ifndef PARTICLEWORLD_HPP_091XQZT6
#define PARTICLEWORLD_HPP_091XQZT6

#include <uniformgrid.h>
#include <managed.hpp>
#include <rbody.hpp>

namespace i3d
{

  template<typename T>
    class ParticleWorld<T,unified> : public Managed
    {
      public:

        T *pos_;
        T *vel_;
        T *force_;

        T *sortedPos_;
        T *sortedVel_;

        int size_;

        SimulationParameters<T> *params_;
        int *type_;

        Rbody<T,unified> *rigidBodies_;


        ParticleWorld() : pos_(nullptr), vel_(nullptr), force_(nullptr),
                          sortedPos_(nullptr), sortedVel_(nullptr), size_(0), 
                          params_(nullptr), type_(nullptr), rigidBodies_(nullptr)
        {

        };

        ParticleWorld(int size) : pos_(nullptr), vel_(nullptr), force_(nullptr),
                                  sortedPos_(nullptr), sortedVel_(nullptr), size_(size), 
                                  params_(nullptr), type_(nullptr), rigidBodies_(nullptr)
        {
          cudaCheck(cudaMallocManaged((void**)&(pos_),   size_ * 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(vel_),   size_ * 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(force_), size_ * 3 * sizeof(T)));

          cudaCheck(cudaMallocManaged((void**)&(sortedPos_), size_ * 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(sortedVel_), size_ * 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(params_), sizeof(SimulationParameters<T>)));
          cudaCheck(cudaMallocManaged((void**)&(type_), size_ * sizeof(int)));

          cudaCheck(cudaMallocManaged((void**)&(rigidBodies_), 2 * sizeof(Rbody<T,unified>)));
        };

        ~ParticleWorld()
        {
          cudaCheck(cudaFree((pos_)));
          cudaCheck(cudaFree((vel_)));
          cudaCheck(cudaFree((force_)));

          cudaCheck(cudaFree((sortedPos_)));
          cudaCheck(cudaFree((sortedVel_)));
          cudaCheck(cudaFree((params_)));
          cudaCheck(cudaFree((type_)));
          cudaCheck(cudaFree((rigidBodies_)));
        }

        void init(int size)
        {
          cudaCheck(cudaMallocManaged((void**)&(pos_), size_* 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(vel_), size_ * 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(force_), size_ * 3 * sizeof(T)));

          cudaCheck(cudaMallocManaged((void**)&(sortedPos_), size_ * 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(sortedVel_), size_ * 4 * sizeof(T)));
          cudaCheck(cudaMallocManaged((void**)&(params_), sizeof(SimulationParameters<T>)));
          cudaCheck(cudaMallocManaged((void**)&(type_), size_ * sizeof(int)));
          cudaCheck(cudaMallocManaged((void**)&(rigidBodies_), 2 * sizeof(Rbody<T,unified>)));
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
