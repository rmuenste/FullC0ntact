
#include <stdio.h>
#include <common.h>
#include <perftimer.h>
#include <cfloat>
#include <difi.cuh>
#include <cutil_math.cuh>
#include <aabb3.h>
#include <boundingvolumetree3.h>
#include <thrust/tuple.h>
#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/for_each.h>
#include <thrust/execution_policy.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/sort.h>
#include <cmath>
#include <worldparameters.h>
#include <uniformgrid.cuh>
#include <ext_unit_tests.cuh>
#include <cuda_runtime_api.h>
#include <cuda.h>
#include <particleworld.hpp>
#include <hashgrid.hpp>

inline float frand()
{
  return rand() / (float)RAND_MAX;
}

inline void cuErr(cudaError_t status, const char *file, const int line)
{
  if(status != cudaSuccess) {
    std::cerr << "Cuda API error: ";
    std::cerr << cudaGetErrorString(status);
    std::cerr << " at line " << line;
    std::cerr << " of file " << file << std::endl;
    std::exit(EXIT_FAILURE); 
  }
}

__global__ void hashgrid_size(HashGrid<float,unified> *g)
{

  printf("HashGrid size = %i\n",g->size_);

  printf("HashGrid number of cells = %i\n",g->numCells_);

  printf("Grid X Y Z = %i %i %i\n",g->gridx_, g->gridy_, g->gridz_);

  printf("Cell size = [%f %f %f]\n" ,g->cellSize_.x
                                    ,g->cellSize_.y
                                    ,g->cellSize_.z);

  printf("Origin = [%f %f %f]\n" ,g->origin_.x
                                 ,g->origin_.y
                                 ,g->origin_.z);

  for(int i(0); i < g->size_; ++i)
  {
    g->hashEntries_[i]=20-i;
    g->particleIndices_[i]=20-i;
    printf("particleIndices_[%i]=%i\n",i, g->particleIndices_[i]);
  }

}

__global__ void d_test_pwparams(ParticleWorld<float,unified> *w)
{

  printf("Number of particles: %i\n",w->params_->numParticles_);
  printf("Spring = %f\n",w->params_->spring_);
  printf("Damping = %f\n",w->params_->damping_);
  printf("Shear = %f\n",w->params_->shear_);
  printf("Attraction = %f\n",w->params_->attraction_);
  printf("Global dampening = %f\n",w->params_->globalDamping_);
  printf("Particle radius = %f\n",w->params_->particleRadius_);
  printf("Gravity = [%f %f %f]\n",w->params_->gravity_.x
                                 ,w->params_->gravity_.y
                                 ,w->params_->gravity_.z);

  for(int i(0); i < w->size_*4; i+=4)
  {
    printf("pos[%i] = [%f %f %f]\n",i/4
                                   ,w->pos_[i]
                                   ,w->pos_[i+1]
                                   ,w->pos_[i+2]
                                   );

    printf("vel[%i] = [%f %f %f]\n",i/4
                                   ,w->vel_[i]
                                   ,w->vel_[i+1]
                                   ,w->vel_[i+2]
                                   );
  }
}

void test_pwparams(ParticleWorld<float,unified> *pw)
{
  d_test_pwparams<<<1,1>>>(pw);
  cudaDeviceSynchronize();
}

void test_initGrid(unsigned *size, float spacing, float jitter, ParticleWorld<float, unified> &pw)
{

  unsigned numParticles = pw.size_;

  float radius = pw.params_->particleRadius_;
  srand(1973);
  for (unsigned z = 0; z<size[2]; z++) {
    for (unsigned y = 0; y<size[1]; y++) {
      for (unsigned x = 0; x<size[0]; x++) {
        unsigned i = (z*size[1] * size[0]) + (y*size[0]) + x;
        if (i < numParticles) {
          pw.pos_[i * 4] = (spacing * x) + radius - 1.0f + (frand()*2.0f - 1.0f)*jitter;
          pw.pos_[i * 4 + 1] = (spacing * y) + radius - 1.0f + (frand()*2.0f - 1.0f)*jitter;
          pw.pos_[i * 4 + 2] = (spacing * z) + radius - 1.0f + (frand()*2.0f - 1.0f)*jitter;
          pw.pos_[i * 4 + 3] = 1.0f;

          pw.vel_[i * 4] = 0.0f;
          pw.vel_[i * 4 + 1] = 0.0f;
          pw.vel_[i * 4 + 2] = 0.0f;
          pw.vel_[i * 4 + 3] = 0.0f;
        }
      }
    }
  }
}

void test_cudaFree()
{
  int *test = nullptr; 
  cudaCheck(cudaFree(test));
  cudaDeviceSynchronize();

  int _size = 10;

  ParticleWorld<float,unified> *myWorld = new ParticleWorld<float,unified>(_size);

  myWorld->size_ = _size;
  myWorld->params_->numParticles_ = 10;
  myWorld->params_->spring_ = 0.5f;
  myWorld->params_->damping_ = 0.02f;
  myWorld->params_->shear_ = 0.1f;
  myWorld->params_->attraction_ = 0.0f;
  myWorld->params_->gravity_ = Vector3<float>(0, 0, -0.0003f);
  myWorld->params_->globalDamping_ = 1.0f;
  myWorld->params_->boundaryDamping_ = -0.5f;
  myWorld->params_->particleRadius_ = 1.0f / 64.0f;
  myWorld->params_->origin_ = Vector3<float>(-1.0f, -1.0f, -1.0f);
  myWorld->params_->gridx_ = 64;
  myWorld->params_->gridy_ = 64;
  myWorld->params_->gridz_ = 64;
  myWorld->params_->timeStep_ = 0.5f;

  float jitter = myWorld->params_->particleRadius_ * 0.01f;
  unsigned int s = (int)std::ceil(std::pow((float)myWorld->size_, 1.0f / 3.0f));

  unsigned int gridSize[3];
  gridSize[0] = gridSize[1] = gridSize[2] = s;
  test_initGrid(gridSize, myWorld->params_->particleRadius_*2.0f, jitter, *myWorld);


  float diameter = 2.0f * myWorld->params_->particleRadius_;
  Vector3<float> cellS(diameter, diameter, diameter);

  HashGrid<float,unified> *grid = new HashGrid<float,unified>(_size,
      myWorld->params_->gridx_,
      myWorld->params_->gridy_,
      myWorld->params_->gridz_,
      cellS,
      myWorld->params_->origin_
      );

  test_pwparams(myWorld);

  hashgrid_size<<<1,1>>>(grid);
  cudaDeviceSynchronize();

  delete myWorld;
  delete grid;

}



