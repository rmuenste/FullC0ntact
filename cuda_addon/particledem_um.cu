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

int _size = 6;

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

//void test_initGrid(unsigned *size, float spacing, float jitter, ParticleWorld<float, unified> &pw)
//{
//
//  unsigned numParticles = pw.size_;
//
//  float radius = pw.params_->particleRadius_;
//  srand(1973);
//  for (unsigned z = 0; z<size[2]; z++) {
//    for (unsigned y = 0; y<size[1]; y++) {
//      for (unsigned x = 0; x<size[0]; x++) {
//        unsigned i = (z*size[1] * size[0]) + (y*size[0]) + x;
//        if (i < numParticles) {
//          pw.pos_[i * 4] = (spacing * x) + radius - 1.0f + (frand()*2.0f - 1.0f)*jitter;
//          pw.pos_[i * 4 + 1] = (spacing * y) + radius - 1.0f + (frand()*2.0f - 1.0f)*jitter;
//          pw.pos_[i * 4 + 2] = (spacing * z) + radius - 1.0f + (frand()*2.0f - 1.0f)*jitter;
//          pw.pos_[i * 4 + 3] = 1.0f;
//
//          pw.vel_[i * 4] = 0.0f;
//          pw.vel_[i * 4 + 1] = 0.0f;
//          pw.vel_[i * 4 + 2] = 0.0f;
//          pw.vel_[i * 4 + 3] = 0.0f;
//        }
//      }
//    }
//  }
//}

void test_initGrid(unsigned *size, float spacing, float jitter, ParticleWorld<float, unified> &pw)
{
  unsigned numParticles = pw.size_;
  size[0]=3;
  size[1]=2;
  size[2]=1;
  float radius = pw.params_->particleRadius_;
  float spacingy = 2.f * spacing;
  srand(1973);
  for (unsigned z = 0; z<size[2]; z++) {
    for (unsigned y = 0; y<size[1]; y++) {
      for (unsigned x = 0; x<size[0]; x++) {
        unsigned i = (z*size[1] * size[0]) + (y*size[0]) + x;
        if (i < numParticles) {
          pw.pos_[i * 4]     = (spacing * x) + radius;// - 1.0f;
          pw.pos_[i * 4 + 1] = (spacingy * y) + radius;// - 1.0f;
          pw.pos_[i * 4 + 2] = (spacing * z) + radius;// - 1.0f;
          pw.pos_[i * 4 + 3] = 1.0f;

          pw.vel_[i * 4] = 0.0f;
          pw.vel_[i * 4 + 1] = 0.0f;
          pw.vel_[i * 4 + 2] = 0.0f;
          pw.vel_[i * 4 + 3] = 0.0f;

          pw.type_[i] = y;
          pw.rigidBodies_[y].indices_[x]=i;
        }
      }
    }
  }

//  printf("body 0 indices: %i %i %i\n",pw.rigidBodies_[0].indices_[0],
//                                      pw.rigidBodies_[0].indices_[1],
//                                      pw.rigidBodies_[0].indices_[2]);
//
//  printf("body 1 indices: %i %i %i\n",pw.rigidBodies_[1].indices_[0],
//                                      pw.rigidBodies_[1].indices_[1],
//                                      pw.rigidBodies_[1].indices_[2]);

//  printf("body 0 type   : %i %i %i\n",pw.type_[0],
//                                      pw.type_[1],
//                                      pw.type_[2]);
//
//  printf("body 1 type   : %i %i %i\n",pw.type_[3],
//                                      pw.type_[4],
//                                      pw.type_[5]);

  pw.rigidBodies_[0].nparticles_=3;
  pw.rigidBodies_[0].com_.x = pw.pos_[1 * 4];
  pw.rigidBodies_[0].com_.y = pw.pos_[1 * 4 + 1];
  pw.rigidBodies_[0].com_.z = pw.pos_[1 * 4 + 2];

  pw.rigidBodies_[0].vel_.x = 0.f; 
  pw.rigidBodies_[0].vel_.y = 0.f;
  pw.rigidBodies_[0].vel_.z = 0.f;

  pw.rigidBodies_[1].nparticles_=3;
  pw.rigidBodies_[1].com_.x = pw.pos_[4 * 4];
  pw.rigidBodies_[1].com_.y = pw.pos_[4 * 4 + 1];
  pw.rigidBodies_[1].com_.z = pw.pos_[4 * 4 + 2];

  pw.rigidBodies_[1].vel_.x = 0.f; 
  pw.rigidBodies_[1].vel_.y = 0.f;
  pw.rigidBodies_[1].vel_.z = 0.f;

}

void buildSphere(ParticleWorld<float, unified> &pw)
{
  unsigned numParticles = pw.size_;

  float radius = pw.params_->particleRadius_;
  int ballr = 10;

  float pr = radius;
  float tr = pr+(pr*2.0f)*ballr;
  float pos[4];
  pos[0] = -1.0f + tr + frand()*(2.0f - tr*2.0f);
  pos[1] = 1.0f - tr;
  pos[2] = -1.0f + tr + frand()*(2.0f - tr*2.0f);
  pos[3] = 0.0f;

  float spacing = pr * 2.0f;
  //psystem->addSphere(0, pos, vel, ballr, pr*2.0f);

  uint index = 0;
  int r = ballr;
  for(int z=-r; z<=r; z++)
  {
    for(int y=-r; y<=r; y++)
    {
      for(int x=-r; x<=r; x++)
      {
        float dx = x*spacing;
        float dy = y*spacing;
        float dz = z*spacing;
        float l = std::sqrt(dx*dx + dy*dy + dz*dz);
        float jitter = radius * 0.01f;
        if ((l <= radius * 2.0f*r) && (index < numParticles)) {
          pw.pos_[index*4]   = pos[0] + dx + (frand()*2.0f-1.0f)*jitter;
          pw.pos_[index*4+1] = pos[1] + dy + (frand()*2.0f-1.0f)*jitter; 
          pw.pos_[index*4+2] = pos[2] + dz + (frand()*2.0f-1.0f)*jitter;
          pw.pos_[index*4+3] = pos[3];

          pw.vel_[index*4]   = 0.0f;
          pw.vel_[index*4+1] = 0.0f;
          pw.vel_[index*4+2] = 0.0f;
          pw.vel_[index*4+3] = 0.0f;
          index++;
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
  //buildSphere(*myWorld);

  float diameter = 2.0f * myWorld->params_->particleRadius_;
  Vector3<float> cellS(diameter, diameter, diameter);

  grid = new HashGrid<float, unified>(_size,
    myWorld->params_->gridx_,
    myWorld->params_->gridy_,
    myWorld->params_->gridz_,
    cellS,
    myWorld->params_->origin_
    );

  std::memset(myWorld->type_, 1, myWorld->size_*sizeof(int));
//  for (unsigned i(0); i < 500; ++i)
//  {
//    myWorld->type_[i] = 0;  
//  }
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

__global__ void setType(ParticleWorld<float, unified> *pw)
{
  pw->type_[0]=0; 
  pw->type_[1]=0;
  pw->type_[2]=0;

  pw->type_[3]=1; 
  pw->type_[4]=1;
  pw->type_[5]=1;
}

void calcHash()
{
   unsigned numThreads, numBlocks;
   computeGridSizeS(myWorld->size_, 256, numBlocks, numThreads);

   // execute the kernel
   calcHashD<<< numBlocks, numThreads >>>(grid, myWorld);
   setType<<<1,1>>>(myWorld);
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

  unsigned originalIndex = hg->particleIndices_[index];

  // get start of bucket for this cell
  uint startIndex = hg->cellStart_[gridHash];

  float3 force = make_float3(0.0f);
  if (startIndex != 0xffffffff)
  {        
    uint endIndex = hg->cellEnd_[gridHash]; 
#ifdef CDEBUG 
    printf("index=%i startIndex[%i]=%i endIndex[%i]=%i\n", index, gridHash, startIndex,
                                                                  gridHash, endIndex);
#endif
    for (uint j = startIndex; j < endIndex; ++j)
    {
      unsigned originalIndexJ = hg->particleIndices_[j];

      //if (j != index)
      if (j != index && pw->type_[originalIndex] != pw->type_[originalIndexJ])
      {              
        float3 pos2 = make_float3(oldPos[j]);
        float3 vel2 = make_float3(oldVel[j]);

        // calculate relative position
        float3 relPos = pos2 - pos;

        float dist = length(relPos);
        float collideDist = pw->params_->particleRadius_ + pw->params_->particleRadius_;
#ifdef CDEBUG 
        //printf("typeA[%i]=%i typeB[%i]=%i\n", originalIndex, pw->type_[originalIndex], originalIndexJ, pw->type_[originalIndexJ]);
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
          float factor = 1.0f;
          sforce = factor * -pw->params_->spring_*(collideDist - dist) * norm;
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
  float3 *_force = (float3*)pw->force_;

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
  _force[originalIndex] = make_float3(force.x, force.y, force.z);

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

__global__ void d_integrateSystem(ParticleWorld<float, unified> *pw)
{

  unsigned int index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;

  if (index >= pw->size_) return;

  // read particle data from sorted arrays
  float4 *d_pos4 = (float4*)pw->pos_;  
  float4 *d_vel4 = (float4*)pw->vel_;


  float3 pos = make_float3(d_pos4[index].x, d_pos4[index].y, d_pos4[index].z);
  float3 vel = make_float3(d_vel4[index].x, d_vel4[index].y, d_vel4[index].z);
  float3 grav = make_float3(pw->params_->gravity_.x,
                            pw->params_->gravity_.y,
                            pw->params_->gravity_.z);

  float deltaTime = pw->params_->timeStep_;

  vel += grav * deltaTime;

  vel *= pw->params_->globalDamping_;

  //if(pw->type_[index]!=0)
  pos += vel * deltaTime;

  if (pos.x >  1.0f - pw->params_->particleRadius_) { pos.x =  1.0f - pw->params_->particleRadius_; vel.x *= pw->params_->boundaryDamping_; }
  if (pos.x < -1.0f + pw->params_->particleRadius_) { pos.x = -1.0f + pw->params_->particleRadius_; vel.x *= pw->params_->boundaryDamping_; }
  if (pos.y >  1.0f - pw->params_->particleRadius_) { pos.y =  1.0f - pw->params_->particleRadius_; vel.y *= pw->params_->boundaryDamping_; }
  if (pos.z >  1.0f - pw->params_->particleRadius_) { pos.z =  1.0f - pw->params_->particleRadius_; vel.z *= pw->params_->boundaryDamping_; }
  if (pos.z < -1.0f + pw->params_->particleRadius_) { pos.z = -1.0f + pw->params_->particleRadius_; vel.z *= pw->params_->boundaryDamping_; }
  if (pos.y < -1.0f + pw->params_->particleRadius_) { pos.y = -1.0f + pw->params_->particleRadius_; vel.y *= pw->params_->boundaryDamping_; }

  // store new position and velocity
  d_pos4[index] = make_float4(pos, d_pos4[index].w);
  d_vel4[index] = make_float4(vel, d_pos4[index].w);

}

__global__ void d_integrateRigidBody(ParticleWorld<float, unified> *pw)
{

  unsigned int index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;

  if (index >= pw->size_) return;

  // read particle data from sorted arrays
  float4 *d_pos4 = (float4*)pw->pos_;  
  float4 *d_vel4 = (float4*)pw->vel_;

  float3 grav = make_float3(pw->params_->gravity_.x,
                            pw->params_->gravity_.y,
                            pw->params_->gravity_.z);

  float deltaTime = pw->params_->timeStep_;

  
  float3 vel = make_float3(pw->rigidBodies_[index].vel_.x,
                           pw->rigidBodies_[index].vel_.y,
                           pw->rigidBodies_[index].vel_.z);

  float3 pos = make_float3(pw->rigidBodies_[index].com_.x,
                           pw->rigidBodies_[index].com_.y,
                           pw->rigidBodies_[index].com_.z);

  vel += grav * deltaTime;

  vel *= pw->params_->globalDamping_;

  //if(pw->type_[index]!=0)
  pos += vel * deltaTime;

  pw->rigidBodies_[index].vel_.z = vel.z; 

  pw->rigidBodies_[index].com_.z = pos.z; 

  if(index == 0)
  {
    float3 pos_ = make_float3(d_pos4[0].x, d_pos4[0].y, d_pos4[0].z);
    pos_ += vel * deltaTime;
    d_pos4[0] = make_float4(pos_, 1.0f);

    pos_ = make_float3(d_pos4[1].x, d_pos4[1].y, d_pos4[1].z);
    pos_ += vel * deltaTime;
    d_pos4[1] = make_float4(pos_, 1.0f);

    pos_ = make_float3(d_pos4[2].x, d_pos4[2].y, d_pos4[2].z);
    pos_ += vel * deltaTime;
    d_pos4[2] = make_float4(pos_, 1.0f);
  }
  else
  {
    float3 pos_ = make_float3(d_pos4[3].x, d_pos4[3].y, d_pos4[3].z);
    pos_ += vel * deltaTime;
    d_pos4[3] = make_float4(pos_, 1.0f);

    pos_ = make_float3(d_pos4[4].x, d_pos4[4].y, d_pos4[4].z);
    pos_ += vel * deltaTime;
    d_pos4[4] = make_float4(pos_, 1.0f);

    pos_ = make_float3(d_pos4[5].x, d_pos4[5].y, d_pos4[5].z);
    pos_ += vel * deltaTime;
    d_pos4[5] = make_float4(pos_, 1.0f);
  }
  

  if (pos.x >  1.0f - pw->params_->particleRadius_) { pos.x =  1.0f - pw->params_->particleRadius_; vel.x *= pw->params_->boundaryDamping_; }
  if (pos.x < -1.0f + pw->params_->particleRadius_) { pos.x = -1.0f + pw->params_->particleRadius_; vel.x *= pw->params_->boundaryDamping_; }
  if (pos.y >  1.0f - pw->params_->particleRadius_) { pos.y =  1.0f - pw->params_->particleRadius_; vel.y *= pw->params_->boundaryDamping_; }
  if (pos.z >  1.0f - pw->params_->particleRadius_) { pos.z =  1.0f - pw->params_->particleRadius_; vel.z *= pw->params_->boundaryDamping_; }
  if (pos.z < -1.0f + pw->params_->particleRadius_) { pos.z = -1.0f + pw->params_->particleRadius_; vel.z *= pw->params_->boundaryDamping_; }
  if (pos.y < -1.0f + pw->params_->particleRadius_) { pos.y = -1.0f + pw->params_->particleRadius_; vel.y *= pw->params_->boundaryDamping_; }

//  // store new position and velocity
//  d_pos4[index] = make_float4(pos, d_pos4[index].w);
//  d_vel4[index] = make_float4(vel, d_pos4[index].w);

}

void integrateSystem()
{

  // thread per particle
  unsigned numThreads, numBlocks;
  unsigned size = _size;
  computeGridSizeS(size, 64, numBlocks, numThreads);

  // execute the kernel
  //d_integrateSystem<<< numBlocks, numThreads >>>(myWorld);
  d_integrateRigidBody<<< 1, 2 >>>(myWorld);

  cudaCheck(cudaDeviceSynchronize());

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
