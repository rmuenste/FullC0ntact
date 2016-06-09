#ifndef PARTICLEDEM_UM_CU_EOLFXAWV
#define PARTICLEDEM_UM_CU_EOLFXAWV

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

ParticleWorld<float, unified> *myWorld;
HashGrid<float, unified> *grid;

int _size = 16384;

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

inline float frand()
{
  return rand() / (float)RAND_MAX;
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

void sortParticles()
{

  cudaDeviceSynchronize();
  thrust::sort_by_key(thrust::cuda::par, thrust::device_ptr<unsigned int>(grid->hashEntries_),
    thrust::device_ptr<unsigned int>(grid->hashEntries_ + grid->size_),
    thrust::device_ptr<unsigned int>(grid->particleIndices_));
  cudaDeviceSynchronize();

}

void cuda_init()
{

  myWorld = new ParticleWorld<float, unified>(_size);

  myWorld->size_ = _size;
  myWorld->params_->numParticles_ = _size;
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

  grid = new HashGrid<float, unified>(_size,
    myWorld->params_->gridx_,
    myWorld->params_->gridy_,
    myWorld->params_->gridz_,
    cellS,
    myWorld->params_->origin_
    );

  cudaDeviceSynchronize();

}

__global__
void calcHashD(HashGrid<float, unified> *hg, ParticleWorld<float, unified> *pw)
{
    unsigned int index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;

    if (index >= pw->size_) return;

    float4 *particlePos = (float4 *)pw->pos_;
    
    float4 p = particlePos[index];

    int3 gridIndex = hg->getGridIndex(make_float3(p.x, p.y, p.z));
    unsigned int hash = hg->hash(gridIndex);

    hg->setParticleHash(hash, index);

#ifdef CDEBUG
    printf("Particle[%i], grid index [%i %i %i], hash=[%i]\n",index, 
                                                              gridIndex.x,
                                                              gridIndex.y,
                                                              gridIndex.z,
                                                              hash);
#endif

}

void computeGridSizeS(unsigned n, unsigned blockSize, unsigned &numBlocks, unsigned &numThreads)
{
    numThreads = std::min(blockSize, n);

    numBlocks = (n % numThreads != 0) ? (n / numThreads + 1) : (n / numThreads);
}

void calcHash()
{
   unsigned numThreads, numBlocks;
   computeGridSizeS(myWorld->size_, 256, numBlocks, numThreads);

   // execute the kernel
   calcHashD<<< numBlocks, numThreads >>>(grid, myWorld);

}

__global__
void reorderDataAndFindCellStartD(HashGrid<float, unified> *hg, ParticleWorld<float, unified> *pw)
{
  extern __shared__ unsigned int sharedHash[];    // blockSize + 1 elements
  unsigned int index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;

  unsigned int hash;
  // handle case when no. of particles not multiple of block size
  if (index < pw->size_) {
    hash = hg->hashEntries_[index];

    // Load hash data into shared memory so that we can look 
    // at neighboring particle's hash value without loading
    // two hash values per thread
    sharedHash[threadIdx.x + 1] = hash;

    if (index > 0 && threadIdx.x == 0)
    {
      // first thread in block must load neighbor particle hash
      sharedHash[0] = hg->hashEntries_[index-1];
    }
  }

  __syncthreads();

  if (index < pw->size_) {
    // If this particle has a different cell index to the previous
    // particle then it must be the first particle in the cell,
    // so store the index of this particle in the cell.
    // As it isn't the first particle, it must also be the cell end of
    // the previous particle's cell

    if (index == 0 || hash != sharedHash[threadIdx.x])
    {
      hg->cellStart_[hash] = index;
      if (index > 0)
        hg->cellEnd_[sharedHash[threadIdx.x]] = index;
    }

    if (index == pw->size_ - 1)
    {
      hg->cellEnd_[hash] = index + 1;
    }

    // Now use the sorted index to reorder the pos and vel data
    unsigned sortedIndex = hg->particleIndices_[index];
    float4 *oldPos = (float4*)pw->pos_;
    float4 *oldVel = (float4*)pw->vel_;
    float4 pos = oldPos[sortedIndex];
    float4 vel = oldVel[sortedIndex];

    float4 *sortedPos = (float4*)pw->sortedPos_;
    float4 *sortedVel = (float4*)pw->sortedVel_;
    sortedPos[index] = pos;
    sortedVel[index] = vel;

  }

}

void reorderDataAndFindCellStart()
{
  unsigned numThreads, numBlocks;
  computeGridSizeS(myWorld->size_, 256, numBlocks, numThreads);
  
  // set all cells to empty
  std::memset(grid->cellStart_, 0xffffffff, grid->numCells_*sizeof(unsigned int));
  std::memset(grid->cellEnd_, 0xffffffff, grid->numCells_*sizeof(unsigned int));

  unsigned smemSize = sizeof(unsigned)*(numThreads + 1);
  reorderDataAndFindCellStartD <<< numBlocks, numThreads, smemSize >>>(grid, myWorld);

}

// collide a particle against all other particles in a given cell
__device__
float3 collideCell(int3 gridPos, uint index, float3 pos, float3 vel, HashGrid<float, unified> *hg, ParticleWorld<float, unified> *pw)
{
  float4 *oldPos = (float4*)pw->sortedPos_;
  float4 *oldVel = (float4*)pw->sortedVel_;
  uint gridHash = hg->hash(gridPos);

  // get start of bucket for this cell
  uint startIndex = hg->cellStart_[gridHash];   //FETCH(cellStart, gridHash);

  float3 force = make_float3(0.0f);
  if (startIndex != 0xffffffff)
  {        
    uint endIndex = hg->cellEnd_[gridHash]; 
#ifdef CDEBUG 
    printf("index=%i startIndex[%i]=%i endIndex[%i]=%i\n", index, gridHash, startIndex,
                                                                  gridHash, endIndex);
#endif
    for (uint j = startIndex; j<endIndex; j++)
    {
      if (j != index)
      {              
        float3 pos2 = make_float3(oldPos[j]);
        float3 vel2 = make_float3(oldVel[j]);

        // calculate relative position
        float3 relPos = pos2 - pos;

        float dist = length(relPos);
        float collideDist = pw->params_->particleRadius_ + pw->params_->particleRadius_;
#ifdef CDEBUG 
        if(index == 7 && j==6)
        {
          printf("index=%i with index=%i dist collide[%f %f] pos2[%f %f %f] pos[%f %f %f]\n",
                  index, j, dist, collideDist,pos2.x,pos2.y,pos2.z, pos.x,pos.y,pos.z);
        }
#endif
        float3 sforce = make_float3(0.0f);
        if (dist < collideDist) {
          float3 norm = relPos / dist;

          // relative velocity
          float3 relVel = vel2 - vel;

          // relative tangential velocity
          float3 tanVel = relVel - (dot(relVel, norm) * norm);
#ifdef CDEBUG 
          if(index == 7 && j == 6)
          {
            printf("index=%i with index=%i tanVel[%f %f %f]\n", index, j, tanVel.x,
                                                                          tanVel.y,
                                                                          tanVel.z);

            printf("index=%i with index=%i dist=%f relPos[%f %f %f]\n", index, j,dist, relPos.x,
                                                                                  relPos.y,
                                                                                  relPos.z);
          }
#endif
          // spring force
          sforce = -pw->params_->spring_*(collideDist - dist) * norm;
          // dashpot (damping) force
          sforce += pw->params_->damping_*relVel;
          // tangential shear force
          sforce += pw->params_->shear_*tanVel;
          // attraction
          sforce += pw->params_->attraction_*relPos;
        }

        force+=sforce;
      }
    }
  }
  return force;
}

__global__
void evalForces_launcher(HashGrid<float, unified> *hg, ParticleWorld<float, unified> *pw)
{
  unsigned int index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;

  if (index >= pw->size_) return;

//  printf("collideD launch index=%i\n",index);

  // read particle data from sorted arrays
  float4 *newVel = (float4*)pw->vel_;
  float4 *oldPos = (float4*)pw->sortedPos_;  
  float4 *oldVel = (float4*)pw->sortedVel_;

  float3 pos = make_float3(oldPos[index].x, oldPos[index].y, oldPos[index].z);
  float3 vel = make_float3(oldVel[index].x, oldVel[index].y, oldVel[index].z);

  // get address in grid
  int3 gridIndex = hg->getGridIndex(pos);

  // examine neighbouring cells
  float3 force = make_float3(0.0f, 0.0f, 0.0f);
  for (int z = -1; z <= 1; z++) {
    for (int y = -1; y <= 1; y++) {
      for (int x = -1; x <= 1; x++) {
        int3 neighbourPos = gridIndex + make_int3(x, y, z);
#ifdef CDEBUG 
        if(index ==7)
        {
          printf("index=%i cell[%i %i %i], neighbour[%i %i %i]\n",
                  index, gridIndex.x, gridIndex.y, gridIndex.z,
                  neighbourPos.x, neighbourPos.y, neighbourPos.z);
        }
#endif
        force += collideCell(neighbourPos, index, pos, vel, hg, pw);
      }
    }
  }

  // write new velocity back to original unsorted location
  unsigned originalIndex = hg->particleIndices_[index];
  float3 vel3 = vel + force;
#ifdef CDEBUG 
  printf("Particle[%i], force[%f %f %f], velocity_new[%f %f %f]\n", index,
                                                                    force.x, force.y, force.z,
                                                                    vel3.x, vel3.y, vel3.z);
#endif
  newVel[originalIndex] = make_float4(vel3.x, vel3.y, vel3.z, 0.0f);

}

void evalForces()
{

  // thread per particle
  unsigned numThreads, numBlocks;
  unsigned size = _size;
  computeGridSizeS(size, 64, numBlocks, numThreads);

  // execute the kernel
  evalForces_launcher <<< numBlocks, numThreads >>>(grid, myWorld);

}

struct integrate_functor
{
  float deltaTime;
  ParticleWorld<float, unified> *world_;

  __host__ __device__
    integrate_functor(float delta_time, ParticleWorld<float, unified> *pw) : deltaTime(delta_time), world_(pw) {}

  template <typename Tuple>
  __host__ __device__
    void operator()(Tuple t)
  {
      volatile float4 posData = thrust::get<0>(t);
      volatile float4 velData = thrust::get<1>(t);
      float3 pos = make_float3(posData.x, posData.y, posData.z);
      float3 vel = make_float3(velData.x, velData.y, velData.z);

      float3 grav = make_float3(world_->params_->gravity_.x, world_->params_->gravity_.y, world_->params_->gravity_.z);
      vel += grav * deltaTime;
      //vel = grav;
      vel *= world_->params_->globalDamping_;

      // new position = old position + velocity * deltaTime
      pos += vel * deltaTime;

      if (pos.x > 1.0f - world_->params_->particleRadius_) { pos.x = 1.0f - world_->params_->particleRadius_; vel.x *= world_->params_->boundaryDamping_; }
      if (pos.x < -1.0f + world_->params_->particleRadius_){ pos.x = -1.0f + world_->params_->particleRadius_; vel.x *= world_->params_->boundaryDamping_; }
      if (pos.y > 1.0f - world_->params_->particleRadius_) { pos.y = 1.0f - world_->params_->particleRadius_; vel.y *= world_->params_->boundaryDamping_; }
      if (pos.z > 1.0f - world_->params_->particleRadius_) { pos.z = 1.0f - world_->params_->particleRadius_; vel.z *= world_->params_->boundaryDamping_; }
      if (pos.z < -1.0f + world_->params_->particleRadius_){ pos.z = -1.0f + world_->params_->particleRadius_; vel.z *= world_->params_->boundaryDamping_; }
      if (pos.y < -1.0f + world_->params_->particleRadius_){ pos.y = -1.0f + world_->params_->particleRadius_; vel.y *= world_->params_->boundaryDamping_; }

      // store new position and velocity
      thrust::get<0>(t) = make_float4(pos, posData.w);
      thrust::get<1>(t) = make_float4(vel, velData.w);
    }
};

void integrateSystem()
{

  cudaDeviceSynchronize();
  thrust::device_ptr<float4> d_pos4((float4 *)myWorld->pos_);
  thrust::device_ptr<float4> d_vel4((float4 *)myWorld->vel_);

  thrust::for_each(
    thrust::make_zip_iterator(thrust::make_tuple(d_pos4, d_vel4)),
    thrust::make_zip_iterator(thrust::make_tuple(d_pos4 + myWorld->size_, d_vel4 + myWorld->size_)),
    integrate_functor(myWorld->params_->timeStep_, myWorld));

}

void transfer_data(std::vector<float> &positions)
{

  cudaDeviceSynchronize();
  for (int i(0); i < 4 * _size; i+=4)
  {
    positions.push_back(myWorld->pos_[i]);
    positions.push_back(myWorld->pos_[i+1]);
    positions.push_back(myWorld->pos_[i+2]);
    positions.push_back(myWorld->pos_[i+3]);
  }

}

#endif /* end of include guard: PARTICLEDEM_UM_CU_EOLFXAWV */
