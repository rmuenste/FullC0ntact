#ifndef PARTICLEDEM_CU_EOLFXAWV
#define PARTICLEDEM_CU_EOLFXAWV

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
//#include <particledem.cuh>

#include <uniformgrid.cuh>

HashGrid<float, i3d::gpu> *d_hashGrid;
ParticleWorld<float, i3d::gpu> *d_particleWorld;

inline float frand()
{
  return rand() / (float)RAND_MAX;
}

void initGrid(unsigned *size,
  float spacing,
  float jitter,
  ParticleWorld<float, cpu> &pw)
{
  unsigned numParticles = pw.size_;
  std::vector<float> pos(numParticles * 4);
  std::vector<float> vel(numParticles * 4);
  float radius = pw.params_->particleRadius_;
  srand(1973);
  for (unsigned z = 0; z<size[2]; z++) {
    for (unsigned y = 0; y<size[1]; y++) {
      for (unsigned x = 0; x<size[0]; x++) {
        unsigned i = (z*size[1] * size[0]) + (y*size[0]) + x;
        if (i < numParticles) {
          pos[i * 4] = (spacing * x) + radius - 1.0f + (frand()*2.0f - 1.0f)*jitter;
          pos[i * 4 + 1] = (spacing * y) + radius - 1.0f + (frand()*2.0f - 1.0f)*jitter;
          pos[i * 4 + 2] = (spacing * z) + radius - 1.0f + (frand()*2.0f - 1.0f)*jitter;
          pos[i * 4 + 3] = 1.0f;

          vel[i * 4] = 0.0f;
          vel[i * 4 + 1] = 0.0f;
          vel[i * 4 + 2] = 0.0f;
          vel[i * 4 + 3] = 0.0f;
        }
      }
    }
  }
  cudaCheck(cudaMemcpy(pw.pos_, pos.data(), numParticles * 4 * sizeof(float), cudaMemcpyHostToDevice));
  cudaCheck(cudaMemcpy(pw.vel_, vel.data(), numParticles * 4 * sizeof(float), cudaMemcpyHostToDevice));
}

unsigned iDivUp(unsigned a, unsigned b){
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

void sortParticles(HashGrid<float, i3d::cpu> &hg)
{
  d_hashGrid->sortParticles(hg.size_, hg.hashEntries_, hg.particleIndices_);
}

void cuda_init(HashGrid<float, cpu> &hg,
               ParticleWorld<float, cpu> &pw,
               WorldParameters &params)
{

  hg.cellSize_.x = 2.0f * pw.params_->particleRadius_;
  hg.cellSize_.y = 2.0f * pw.params_->particleRadius_;
  hg.cellSize_.z = 2.0f * pw.params_->particleRadius_;

  hg.origin_ = pw.params_->origin_;

  hg.gridx_ = pw.params_->gridx_;
  hg.gridy_ = pw.params_->gridy_;
  hg.gridz_ = pw.params_->gridz_;

  hg.numCells_ = hg.gridx_ * hg.gridy_ * hg.gridz_;

  cudaCheck(cudaMalloc((void**)&d_hashGrid, sizeof(HashGrid<float, gpu>)));

  cudaCheck(cudaMemcpy(d_hashGrid, &hg, sizeof(HashGrid<float, gpu>), cudaMemcpyHostToDevice));

  d_hashGrid->initGrid(hg);

  cudaCheck(cudaMalloc((void**)&d_particleWorld, sizeof(ParticleWorld<float, gpu>)));
  cudaCheck(cudaMemcpy(d_particleWorld, &pw, sizeof(ParticleWorld<float, gpu>), cudaMemcpyHostToDevice));

  d_particleWorld->initData(pw);

  float jitter = pw.params_->particleRadius_ * 0.01f;
  unsigned int s = (int)std::ceil(std::pow((float)pw.size_, 1.0f / 3.0f));
  unsigned int gridSize[3];
  gridSize[0] = gridSize[1] = gridSize[2] = s;
  initGrid(gridSize, pw.params_->particleRadius_*2.0f, jitter, pw);

}

__global__
void calcHashD(HashGrid<float,gpu> *hg, ParticleWorld<float,gpu> *pw)
{
    unsigned int index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;

    if (index >= pw->size_) return;

    float4 *particlePos = (float4 *)pw->pos_;
    
    float4 p = particlePos[index];

    int3 gridIndex = hg->getGridIndex(make_float3(p.x, p.y, p.z));
    unsigned int hash = hg->hash(gridIndex);

    hg->setParticleHash(hash, index);

//    printf("Particle[%i], grid index [%i %i %i], hash=[%i]\n",index, 
//                                                              gridIndex.x,
//                                                              gridIndex.y,
//                                                              gridIndex.z,
//                                                              hash);

}

void computeGridSize(unsigned n, unsigned blockSize, unsigned &numBlocks, unsigned &numThreads)
{
    numThreads = std::min(blockSize, n);
    numBlocks = iDivUp(n, numThreads);
}

void calcHash(HashGrid<float,cpu> &hg, ParticleWorld<float,cpu> &pw)
{
   unsigned numThreads, numBlocks;
   computeGridSize(pw.size_, 256, numBlocks, numThreads);

   // execute the kernel
   calcHashD<<< numBlocks, numThreads >>>(d_hashGrid, d_particleWorld);
}

__global__
void reorderDataAndFindCellStartD(HashGrid<float, gpu> *hg, ParticleWorld<float, gpu> *pw)
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

void reorderDataAndFindCellStart(HashGrid<float, cpu> &hg, ParticleWorld<float, cpu> &pw)
{
  unsigned numThreads, numBlocks;
  computeGridSize(pw.size_, 256, numBlocks, numThreads);

  // set all cells to empty
  cudaCheck(cudaMemset(hg.cellStart_, 0xffffffff, hg.numCells_*sizeof(unsigned int)));
  cudaCheck(cudaMemset(hg.cellEnd_, 0xffffffff, hg.numCells_*sizeof(unsigned int)));

  unsigned smemSize = sizeof(unsigned)*(numThreads + 1);
  reorderDataAndFindCellStartD <<< numBlocks, numThreads, smemSize >>>(d_hashGrid, d_particleWorld);
}

// collide two spheres using DEM method
__device__
float3 collideSpheres(float3 posA, float3 posB,
float3 velA, float3 velB,
float radiusA, float radiusB,
float attraction, ParticleWorld<float, gpu> *pw)
{
  // calculate relative position
  float3 relPos = posB - posA;

  float dist = length(relPos);
  float collideDist = radiusA + radiusB;

  float3 force = make_float3(0.0f);
  if (dist < collideDist) {
    float3 norm = relPos / dist;

    // relative velocity
    float3 relVel = velB - velA;

    // relative tangential velocity
    float3 tanVel = relVel - (dot(relVel, norm) * norm);

    // spring force
    
    force = -pw->params_->spring_*(collideDist - dist) * norm;
    // dashpot (damping) force
    force += pw->params_->damping_*relVel;
    // tangential shear force
    force += pw->params_->shear_*tanVel;
    // attraction
    force += attraction*relPos;
  }

  return force;
}

// collide a particle against all other particles in a given cell
__device__
float3 collideCell(int3 gridPos, uint index, float3 pos, float3 vel, HashGrid<float, gpu> *hg, ParticleWorld<float, gpu> *pw)
//uint    index,
//float3  pos,
//float3  vel,
//float4* oldPos,
//float4* oldVel,
//uint*   cellStart,
//uint*   cellEnd)
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
//    printf("index=%i startIndex[%i]=%i endIndex[%i]=%i\n", index, gridHash, startIndex,
//                                                                  gridHash, endIndex);
    for (uint j = startIndex; j<endIndex; j++)
    {
      if (j != index)
      {              
        float3 pos2 = make_float3(oldPos[j]);
        float3 vel2 = make_float3(oldVel[j]);

//        float3 sforce = collideSpheres(pos, pos2, vel, vel2, 
//                                pw->params_->particleRadius_, 
//                                pw->params_->particleRadius_, 
//                                pw->params_->attraction_, pw);

        // calculate relative position
        float3 relPos = pos2 - pos;

        float dist = length(relPos);
        float collideDist = pw->params_->particleRadius_ + pw->params_->particleRadius_;
//        if(index == 7 && j==6)
//        {
//          printf("index=%i with index=%i dist collide[%f %f] pos2[%f %f %f] pos[%f %f %f]\n",
//                  index, j, dist, collideDist,pos2.x,pos2.y,pos2.z, pos.x,pos.y,pos.z);
//        }

        float3 sforce = make_float3(0.0f);
        if (dist < collideDist) {
          float3 norm = relPos / dist;

          // relative velocity
          float3 relVel = vel2 - vel;

          // relative tangential velocity
          float3 tanVel = relVel - (dot(relVel, norm) * norm);
//          if(index == 7 && j == 6)
//          {
//            printf("index=%i with index=%i tanVel[%f %f %f]\n", index, j, tanVel.x,
//                                                                          tanVel.y,
//                                                                          tanVel.z);
//
//            printf("index=%i with index=%i dist=%f relPos[%f %f %f]\n", index, j,dist, relPos.x,
//                                                                                  relPos.y,
//                                                                                  relPos.z);
//
//          }
          // spring force
          sforce = -pw->params_->spring_*(collideDist - dist) * norm;
          // dashpot (damping) force
          sforce += pw->params_->damping_*relVel;
          // tangential shear force
          sforce += pw->params_->shear_*tanVel;
          // attraction
          sforce += pw->params_->attraction_*relPos;
//          if(index == 7 && j == 6)
//          {
//            printf("index=%i with index=%i force[%f %f %f]\n", index, j, sforce.x,
//                                                                         sforce.y,
//                                                                         sforce.z);
//          }
        }

        force+=sforce;
      }
    }
  }
  else
  {
//    if(index ==7)
//    {
//      printf("index=%i cell[%i %i %i] hash=%i cellStart=%i no entry\n",
//              index, gridPos.x, gridPos.y, gridPos.z,gridHash, startIndex);
//
//    }
  }
  return force;
}

__global__
void collideD(HashGrid<float, gpu> *hg, ParticleWorld<float, gpu> *pw)
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
//        if(index ==7)
//        {
//          printf("index=%i cell[%i %i %i], neighbour[%i %i %i]\n",
//                  index, gridIndex.x, gridIndex.y, gridIndex.z,
//                  neighbourPos.x, neighbourPos.y, neighbourPos.z);
//        }

        force += collideCell(neighbourPos, index, pos, vel, hg, pw);
      }
    }
  }

  // write new velocity back to original unsorted location
  unsigned originalIndex = hg->particleIndices_[index];
  float3 vel3 = vel + force;
//  printf("Particle[%i], force[%f %f %f], velocity_new[%f %f %f]\n", index,
//                                                                           force.x, force.y, force.z,
//                                                                           vel3.x, vel3.y, vel3.z);
  newVel[originalIndex] = make_float4(vel3.x, vel3.y, vel3.z, 0.0f);
  //printf("Particle[%i], velocity[%f %f %f]\n", index, vel3.x, vel3.y, vel3.z);

}

void collide(HashGrid<float, cpu> &hg, ParticleWorld<float, cpu> &pw)
{

  // thread per particle
  unsigned numThreads, numBlocks;
  computeGridSize(pw.size_, 64, numBlocks, numThreads);

  // execute the kernel
  collideD <<< numBlocks, numThreads >>>(d_hashGrid, d_particleWorld);

}

struct integrate_functor
{
  float deltaTime;
  ParticleWorld<float, gpu> *world_;

  __host__ __device__
    integrate_functor(float delta_time, ParticleWorld<float, gpu> *pw) : deltaTime(delta_time), world_(pw) {}

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

void integrateSystem(float *pos, float *vel, float deltaTime, uint numParticles)
{
  thrust::device_ptr<float4> d_pos4((float4 *)pos);
  thrust::device_ptr<float4> d_vel4((float4 *)vel);

  thrust::for_each(
    thrust::make_zip_iterator(thrust::make_tuple(d_pos4, d_vel4)),
    thrust::make_zip_iterator(thrust::make_tuple(d_pos4 + numParticles, d_vel4 + numParticles)),
    integrate_functor(deltaTime, d_particleWorld));

}


#endif /* end of include guard: PARTICLEDEM_CU_EOLFXAWV */
