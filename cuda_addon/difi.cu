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


struct printf_functor
{
  __host__ __device__
  void operator()(int x)
  {
      // note that using printf in a __device__ function requires
      // code compiled for a GPU with compute capability 2.0 or
      // higher (nvcc --arch=sm_20)
      printf("%d\n", x);
  }
};

void test_iters()
{
  thrust::device_vector<int> d_vec(3);
  d_vec[0] = 0; d_vec[1] = 1; d_vec[2] = 2;
  thrust::for_each(thrust::device, d_vec.begin(), d_vec.end(), printf_functor());
}

void cudaCheckError(const char *message, const char *file, const int line)
{
  cudaError_t err = cudaGetLastError();
  if (cudaSuccess != err)
  {
    fprintf(stderr, "cudaCheckError() failed at %s:%i : %s User error message: %s\n",
        file, line, cudaGetErrorString(err), message);
    exit(-1);
  }

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

int g_triangles;
int g_verticesGrid;

const int NN = 100 * 1000;

Model3D *g_model;

triangle *d_triangles;
vector3 *d_vertices;
vector3 *d_vertices_grid;

AABB3f *d_boundingBoxes;

vector3 *d_vertexCoords;
vector3 *d_normals;
vector3 *d_contactPoints;      

float *d_distance_map;

int *d_inout;
real *d_distance;


#include "bvh.cuh"
#include "intersection.cuh"
#include "distance.cuh"


#include "distancemap.cuh"
#include "collidermeshmesh.cuh"
#include "uniformgrid.cuh"
#include "unit_tests.cuh"

BVHNode<float> *d_nodes;

DMap *d_map;

DistanceMap<float,gpu> *d_map_gpu;

UniformGrid<float,ElementCell,VertexTraits<float>,gpu> *d_unigrid_gpu;

//HashGrid<float, i3d::gpu> *d_hashGrid;
//ParticleWorld<float, i3d::gpu> *d_particleWorld;

std::vector< DistanceMap<float, gpu>* >  d_maps_gpu;

#include "auxiliary_functions.cuh"

__global__ void outputHashGrid(HashGrid<float,gpu> *g)
{
  g->outputInfo();
}

void hashgrid_sort()
{
  //return;
}

void copy_data(i3d::HashGrid<float, i3d::cpu> &hg, 
               i3d::ParticleWorld<float, i3d::cpu> &pw,
               std::vector<float> &pos)
{

  cudaCheck(cudaMemcpy(pos.data(),
                       pw.pos_,
                       pw.size_ * 4 * sizeof(float),
                       cudaMemcpyDeviceToHost)); 

}

__global__ void hashgrid_size(HashGrid<float,gpu> *g)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

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

__global__ void output_sorted(HashGrid<float, gpu> *g, ParticleWorld<float,gpu> *w)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  float4 *pos_ = (float4*)w->pos_;  
  float4 *pos_sorted = (float4*)w->sortedPos_;  


  for (int i(0); i < g->size_; ++i)
  {
    printf(" hashEntries_[%i]=%i particleIndices_[%i]=%i \n", i, g->hashEntries_[i], i, g->particleIndices_[i]);

    printf("pos_unsorted = [%f %f %f]\n" ,pos_[i].x
                                         ,pos_[i].y
                                         ,pos_[i].z);

    printf("pos_sorted = [%f %f %f]\n" ,pos_sorted[i].x
                                       ,pos_sorted[i].y
                                       ,pos_sorted[i].z);

  }

}

__global__ void test_particleworld(ParticleWorld<float,gpu> *w)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

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

__global__ void output_cellstart(HashGrid<float, gpu> *g)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  for (int i(0); i < g->size_; ++i)
  {
    //printf(" cellStart_[%i]=%i cellEnd_[%i]=%i \n", i, g->cellStart_[i], i, g->cellEnd_[i]);
    printf("hash=%i cellStart_[%i]=%i cellEnd_[%i]=%i \n", g->hashEntries_[i], g->hashEntries_[i], 
                                                    g->cellStart_[g->hashEntries_[i]],
                                                    g->hashEntries_[i], 
                                                    g->cellEnd_[g->hashEntries_[i]]);
    
  }

}


//void test_hashgrid(HashGrid<float, cpu> &hg, ParticleWorld<float, cpu> &pw,
//    WorldParameters &params)
//{
//  hg.size_ = 10;
//  pw.size_ = hg.size_;
//
//  hg.cellSize_.x = 2.0f * pw.params_->particleRadius_; 
//  hg.cellSize_.y = 2.0f * pw.params_->particleRadius_; 
//  hg.cellSize_.z = 2.0f * pw.params_->particleRadius_; 
//
//  hg.origin_   = pw.params_->origin_;
//
//  hg.gridx_ = pw.params_->gridx_;
//  hg.gridy_ = pw.params_->gridy_;
//  hg.gridz_ = pw.params_->gridz_;
//
//  hg.numCells_ = hg.gridx_ * hg.gridy_ * hg.gridz_;  
//
//  cudaCheck(cudaMalloc((void**)&d_hashGrid, sizeof(HashGrid<float,gpu>)));
//
//  cudaCheck(cudaMemcpy(d_hashGrid, &hg, sizeof(HashGrid<float,gpu>), cudaMemcpyHostToDevice));
//
//  d_hashGrid->initGrid(hg);
//
//  hashgrid_size<<<1,1>>>(d_hashGrid);
//  cudaDeviceSynchronize();
//
//  //d_hashGrid->sortGrid(hg.particleIndices_); 
//  //outputHashGrid<<<1,1>>>(d_hashGrid);
//  cudaDeviceSynchronize();
//
//  cudaCheck(cudaMalloc((void**)&d_particleWorld, sizeof(ParticleWorld<float,gpu>)));
//  cudaCheck(cudaMemcpy(d_particleWorld, &pw, sizeof(ParticleWorld<float,gpu>), cudaMemcpyHostToDevice));
//
//  d_particleWorld->initData(pw);
//  cudaDeviceSynchronize();
//
//  float jitter = pw.params_->particleRadius_ * 0.01f;
//  unsigned int s = (int) std::ceil(std::pow((float) pw.size_, 1.0f / 3.0f));
//  unsigned int gridSize[3];
//  gridSize[0] = gridSize[1] = gridSize[2] = s;
//  initGrid(gridSize, pw.params_->particleRadius_*2.0f, jitter, pw);
//  cudaDeviceSynchronize();
//
//  calcHash(hg, pw);
//  cudaDeviceSynchronize();
//
//  d_hashGrid->sortParticles(hg.size_, hg.hashEntries_, hg.particleIndices_);
//  cudaDeviceSynchronize();
//
//  output_sorted <<< 1, 1 >>> (d_hashGrid, d_particleWorld);
//  cudaDeviceSynchronize();
//
//  reorderDataAndFindCellStart(hg, pw);
//  cudaDeviceSynchronize();
//
//  output_cellstart <<< 1, 1 >>>(d_hashGrid);
//  cudaDeviceSynchronize();
//
//  collide(hg, pw);
//  cudaDeviceSynchronize();
//
//  float timestep = pw.params_->timeStep_;
//
//  integrateSystem(pw.pos_, pw.vel_, timestep, pw.size_);
//  cudaDeviceSynchronize();
//
//  test_particleworld<<<1, 1 >>>(d_particleWorld);
//  cudaDeviceSynchronize();
//
//}

//void test_hashgrid2(HashGrid<float, cpu> &hg, ParticleWorld<float, cpu> &pw,
//    WorldParameters &params)
//{
//
////  hg.size_ = 10;
////  pw.size_ = hg.size_;
////
////  hg.cellSize_.x = 2.0f * pw.params_->particleRadius_;
////  hg.cellSize_.y = 2.0f * pw.params_->particleRadius_;
////  hg.cellSize_.z = 2.0f * pw.params_->particleRadius_;
////
////  hg.origin_ = pw.params_->origin_;
////
////  hg.gridx_ = pw.params_->gridx_;
////  hg.gridy_ = pw.params_->gridy_;
////  hg.gridz_ = pw.params_->gridz_;
////
////  hg.numCells_ = hg.gridx_ * hg.gridy_ * hg.gridz_;
////
////  cudaCheck(cudaMalloc((void**)&d_hashGrid, sizeof(HashGrid<float, gpu>)));
////
////  cudaCheck(cudaMemcpy(d_hashGrid, &hg, sizeof(HashGrid<float, gpu>), cudaMemcpyHostToDevice));
////
////  d_hashGrid->initGrid(hg);
////
////  cudaCheck(cudaMalloc((void**)&d_particleWorld, sizeof(ParticleWorld<float, gpu>)));
////  cudaCheck(cudaMemcpy(d_particleWorld, &pw, sizeof(ParticleWorld<float, gpu>), cudaMemcpyHostToDevice));
////
////  d_particleWorld->initData(pw);
////
////  float jitter = pw.params_->particleRadius_ * 0.01f;
////  unsigned int s = (int)std::ceil(std::pow((float)pw.size_, 1.0f / 3.0f));
////  unsigned int gridSize[3];
////  gridSize[0] = gridSize[1] = gridSize[2] = s;
////  initGrid(gridSize, pw.params_->particleRadius_*2.0f, jitter, pw);
//
//  calcHash(hg, pw);
//
//  d_hashGrid->sortParticles(hg.size_, hg.hashEntries_, hg.particleIndices_);
//
//  reorderDataAndFindCellStart(hg, pw);
//
////  cudaDeviceSynchronize();
////  cudaCheckErrors("reorder test");
////  output_sorted<<<1,1>>>(d_hashGrid, d_particleWorld);
////  output_cellstart<<<1,1>>>(d_hashGrid);
////  cudaDeviceSynchronize();
////  return;
//
//  collide(hg, pw);
//
////  output_sorted<<<1,1>>>(d_hashGrid, d_particleWorld);
////  output_cellstart<<<1,1>>>(d_hashGrid);
////  cudaDeviceSynchronize();
//  
//  //test_particleworld<<<1, 1 >>>(d_particleWorld);
//  //cudaDeviceSynchronize();
//  //cudaCheckErrors("kernel test");
//
//  float timestep = pw.params_->timeStep_;
//
//  integrateSystem(pw.pos_, pw.vel_, timestep, pw.size_);
//  //test_particleworld<<<1, 1 >>>(d_particleWorld);
//  cudaDeviceSynchronize();
//
//}

void all_points_dist(UnstructuredGrid<Real, DTraits> &grid)
{

  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);
  d_points_dist << < (grid.nvt_ + 1023) / 1024, 1024 >> > (d_vertices_grid, d_triangles, d_vertices, d_boundingBoxes, d_distance);
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float elapsed_time;
  cudaEventElapsedTime(&elapsed_time, start, stop);
  cudaDeviceSynchronize();
  printf("GPU time distance: %3.8f [ms]\n", elapsed_time);
  real *mydist = new real[grid.nvt_];
  cudaMemcpy(mydist, d_distance, grid.nvt_ * sizeof(real), cudaMemcpyDeviceToHost);
  int id = 0;
  for (id = 0; id < grid.nvt_; id++)
  {
    if (grid.m_myTraits[id].iTag)
    {
      grid.m_myTraits[id].distance = -1.0f * sqrtf(mydist[id]);
      grid.m_myTraits[id].distance = sqrtf(mydist[id]);
    }
    else
      grid.m_myTraits[id].distance = sqrtf(mydist[id]);
  }
  cudaDeviceSynchronize();

}

__global__ void test_distmap(DistanceMap<float,gpu> *map)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(idx < map->dim_[0]*map->dim_[1])
  {
    vector3 query = map->vertexCoords_[idx];
    query += vector3(0.1,0,0); 
    vector3 cp(0,0,0);
    float dist=0;
    dist=dist+1;
    map->queryMap(query,dist,cp);
  }

}

__global__ void sphere_gpu(UniformGrid<float,ElementCell,VertexTraits<float>,gpu> *g,Sphere<float> *spheres,int s)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(idx < s)
  {
    g->queryVertex(spheres[idx]);
  }

}
__global__ void eval_distmap_kernel(DistanceMap<float, gpu> *map,
    vector3 *v, vector3 *cps, vector3 *normals,
    float *distance, int size,
    TransInfo info)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;


  if (idx < size)
  {
    float dist(1000.0f);
    vector3 query_w = v[idx];
    query_w = info.m2w1 * query_w + info.origin1;
    vector3 query = info.w2m0 * (query_w - info.origin0);
    vector3 cp(0, 0, 0);
    if(!map->boundingBox_.isPointInside(query))
    {
      distance[idx] = 1000.0f;
      return;
      //        printf("vertexCoords = %f %f %f inside\n", v[idx].x, v[idx].y, v[idx].z);               
    }
    map->queryMap(query, dist, cp);
    //      printf("transformed_cpu = %f %f %f = %f\n", query.x, query.y, query.z, dist);               

    //printf("dist : %f v0: %f %f %f\n",dist,info.origin0.x,info.origin0.y,info.origin0.z);
    //printf("cp_on_gpu : %f v0: %f %f %f\n",dist,info.origin0.x,info.origin0.y,info.origin0.z);
    //      for(int j(0); j < 9; ++j)
    //        printf("info.m2w0: %f \n",info.m2w0.m_dEntries[j]);

    // transform the contact point cp into world space
    vector3 c0 = (info.m2w0 * cp) + info.origin0;

    // calculate a normal in world space
    normals[idx] = c0 - query_w;
    normals[idx].normalize();

    // calculate the final contact point as the average
    cp = 0.5f * (c0 + query_w);
    distance[idx] = dist;
    cps[idx] = cp;
    //      else
    //      {
    //        printf("vertexCoords = %f %f %f outside\n", v[idx].x, v[idx].y, v[idx].z);               
    //        map->info();
    //      }
  }
}

void eval_distmap(DistanceMap<float, gpu> *map, vector3 *v, 
    vector3 *cps, vector3 *normals,
    float *distance, int size,
    TransInfo info)
{

  const int tpb = 512;
  int blocks = (size+tpb-1)/tpb;
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);

  eval_distmap_kernel<<< blocks, tpb >>>(map, v, cps, normals, distance, size, info); 
  //eval_distmap_kernel<<< 1, 1 >>>(map, v, cps, normals, distance, size, info); 
  //eval_distmap_kernel<<< 1, 1 >>>(map, v, distance, size, info); 

  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float elapsed_time;
  cudaEventElapsedTime(&elapsed_time, start, stop);
  cudaDeviceSynchronize();
  printf("> Elapsed time gpu distmap: %3.8f [ms].\n", elapsed_time);
  cudaCheckErrors("eval_distmap");
}

__global__ void dmap_kernel(UniformGrid<float,ElementCell,VertexTraits<float>,gpu> *g,
    DistanceMap<float,gpu> *map, vector3 com, Mat3f m)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(idx < map->dim_[0]*map->dim_[1])
  {
    vector3 query = g->traits_.vertexCoords_[idx] - com;
    query = m * query;
    vector3 cp(0,0,0);
    g->traits_.fbmVertices_[idx] = map->queryFBM(query);
  }

}

__global__ void queryGrid(UniformGrid<float,ElementCell,VertexTraits<float>,gpu> *g, int j)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(g->traits_.fbmVertices_[j])
    printf("fbm_vertex = %i %i \n", j, g->traits_.fbmVertices_[j]);

}

__global__ void copyFBM(UniformGrid<float,ElementCell,VertexTraits<float>,gpu> *g, int *d, int size)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(idx < size)
  {
    d[idx] = g->traits_.fbmVertices_[idx];
  }

}

void sphere_test(RigidBody *body, UniformGrid<Real,ElementCell,VertexTraits<Real>> &grid)
{

  int size = body->map_->dim_[0] * body->map_->dim_[1];

  vector3 *testVectors = new vector3[NN];
  vector3 *d_testVectors;

  Real *distance_res = new Real[NN];
  float *d_distance_res;
  float *distance_gpu = new float[NN];

  std::vector<Sphere<float>> spheres;

  for(int i=0; i < NN; i++)
  {
    vector3 vr(0,0,0);

    vr.x = -body->map_->boundingBox_.extents_[0] + frand() *
      (2.0 * body->map_->boundingBox_.extents_[0]); 

    vr.y = -body->map_->boundingBox_.extents_[1] + frand() *
      (2.0 * body->map_->boundingBox_.extents_[1]); 

    vr.z = -body->map_->boundingBox_.extents_[2] + frand() *
      (2.0 * body->map_->boundingBox_.extents_[2]); 

    testVectors[i] = vr;
  }

  cudaMalloc((void**)&(d_testVectors), NN*sizeof(vector3));
  cudaMemcpy(d_testVectors, testVectors, NN*sizeof(vector3), cudaMemcpyHostToDevice);

  cudaMalloc((void**)&(d_distance_res), NN*sizeof(float));

  int vx = grid.traits_.cells_[0]+1;
  int vxy=vx*vx*vx;
  printf("CPU distmap for %i points\n", vxy);

  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);

  test_dist<<< (vxy+255)/256, 256 >>>(body->map_gpu_,d_testVectors, d_distance_res);
  cudaMemcpy(distance_gpu, d_distance_res, NN*sizeof(float), cudaMemcpyDeviceToHost);
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float elapsed_time;
  cudaEventElapsedTime(&elapsed_time, start, stop);
  cudaDeviceSynchronize();
  printf("Elapsed time gpu distmap: %3.8f [ms].\n", elapsed_time);
  float gpu_distmap = elapsed_time;

  CPerfTimer timer;
  timer.Start();
  //  for (int i = 0; i < vxy; i++)
  //  {
  //    Vector3<Real> v=grid.traits_.vertexCoords_[i];
  //    std::pair<Real, Vector3<Real>> res = body->map_->queryMap(v);
  //    grid.traits_.fbmVertices_[i] = (res.first < 0.0) ? 1 : 0;
  //  }

  float cpu_distmap = timer.GetTime();
  std::cout << "Elapsed time cpu distmap: " <<  cpu_distmap << " [ms]." << std::endl;

  cudaDeviceSynchronize();

  grid.queryVertex(body->spheres[185]);

  printf("Inner sphere test with %i spheres\n", int(body->spheres.size()));

  timer.Start();

  //  for(auto &sphere : body->spheres)
  //  {
  //    grid.query(sphere);
  //  }

  float cpu_spheres = timer.GetTime();
  std::cout << "Elapsed time cpu spheres: " <<  cpu_spheres << " [ms]." << std::endl;

  for (int i = 0; i < vxy; i++)
  {
    grid.traits_.fbmVertices_[i] = 0;
  }

  timer.Start();

  for(auto &sphere : body->spheres)
  {
    grid.queryVertex(sphere);
  }

  float cpu_spheres_vertex = timer.GetTime();
  std::cout << "Elapsed time cpu spheres[vertex]: " <<  cpu_spheres_vertex << " [ms]." << std::endl;

  for(auto &sphere : body->spheres)
  {
    Sphere<float> s(sphere); 
    spheres.push_back(s);
  }

  Sphere<float> *dev_spheres;
  cudaMalloc((void**)&(dev_spheres), spheres.size() * sizeof(Sphere<float>));
  cudaMemcpy(dev_spheres, spheres.data(), spheres.size()*sizeof(Sphere<float>), cudaMemcpyHostToDevice);

  cudaEvent_t start0, stop0;
  cudaEventCreate(&start0);
  cudaEventCreate(&stop0);
  cudaEventRecord(start0, 0);

  sphere_gpu<<< (spheres.size()+255)/256, 256 >>>(d_unigrid_gpu,dev_spheres,spheres.size());

  cudaEventRecord(stop0, 0);
  cudaEventSynchronize(stop0);
  float elapsed_time0;
  cudaEventElapsedTime(&elapsed_time0, start0, stop0);
  cudaDeviceSynchronize();
  printf("Elapsed time gpu spheres[vertex]: %3.8f [ms].\n", elapsed_time0);

  int size2 = (grid.m_iDimension[0]+1) * (grid.m_iDimension[1]+1) * (grid.m_iDimension[2]+1);
  int *fbmVertices = new int[size2];
  int *dev_fbmVertices;
  cudaMalloc((void**)&(dev_fbmVertices), size2 * sizeof(int));

  copyFBM<<< (size2+255)/256, 256 >>>(d_unigrid_gpu,dev_fbmVertices,size2);

  cudaMemcpy(fbmVertices, dev_fbmVertices,
      size2 * sizeof(int), cudaMemcpyDeviceToHost);
  cudaDeviceSynchronize();

  int gpuIn=0;
  int cpuIn=0;
  for (int i = 0; i < size2; i++)
  {
    if(fbmVertices[i] != grid.traits_.fbmVertices_[i])
    {
      printf("gpu = %i cpu = %i %i\n", fbmVertices[i], grid.traits_.fbmVertices_[i],i);
    }
    if(fbmVertices[i])
    {
      gpuIn++;
    }
    if(grid.traits_.fbmVertices_[i])
    {
      cpuIn++;
    }
  }

  printf("gpuIn = %i \n", gpuIn);
  printf("cpuIn = %i \n", cpuIn);

  printf("cpu_distmap = %3.8f \n", cpu_distmap);
  printf("gpu_distmap = %3.8f \n", gpu_distmap);
  printf("cpu_spheres = %3.8f \n", cpu_spheres_vertex);
  printf("gpu_distmap = %3.8f \n", elapsed_time0);

  delete[] testVectors;
  delete[] distance_res;
  delete[] distance_gpu;
  delete[] fbmVertices;

  cudaFree(d_testVectors);
  cudaFree(d_distance_res);
  cudaFree(dev_fbmVertices);

}

__global__ void test_grid(UniformGrid<float,ElementCell,VertexTraits<float>,gpu> *g,
    int dimx, int dimy, int dimz)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  {
    if(dimx == g->dim_[0])
    {
      printf("> Test 1: size OK!\n");
    }
    else
    {
      printf("> Test 1: size not OK, failed!\n");
    }

    if(dimx*dimy*dimz == g->dim_[0]*g->dim_[1]*g->dim_[2])
    {
      printf("> Test 2: total size OK!\n");
    }
    else
    {
      printf("> Test 1: total size not OK, failed!\n");
    }

  }

}

void transfer_uniformgrid(UniformGrid<Real,ElementCell,VertexTraits<Real>> *grid)
{

  UniformGrid<float,ElementCell,VertexTraits<float>,cpu> grid_(grid);

  cudaMalloc((void**)&(d_unigrid_gpu), sizeof(UniformGrid<float,ElementCell,VertexTraits<float>,gpu>));
  cudaCheckErrors("Allocate uniform grid");

  cudaMemcpy(d_unigrid_gpu, &grid_, sizeof(UniformGrid<float,ElementCell,VertexTraits<float>,cpu>), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy uniformgrid class");

  d_unigrid_gpu->transferData(grid_);

  test_grid<<<1,1>>>(d_unigrid_gpu, grid->m_iDimension[0],
      grid->m_iDimension[1],
      grid->m_iDimension[2]);
  cudaDeviceSynchronize();

}

__global__ void dmap_kernel_test(DistanceMap<float,gpu> *map, UniformGrid<float,ElementCell,VertexTraits<float>,gpu> *g)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(idx < map->dim_[0]*map->dim_[1])
  {
    vector3 query(0,0.05,0);
    vector3 cp(0,0,0);
    float dist=0;
    //printf("fbm_vertex = %i %i \n", j, g->traits_.fbmVertices_[j]);
    vector3 v = g->traits_.vertexCoords_[16000];
    printf(" vertexCoords[16000] = %f %f %f\n", v.x, v.y, v.z);
    int result = map->queryFBM(query);
    printf("idx = %d vertexCoords = %f %f %f = %d\n", idx, query.x, query.y, query.z, result);               
  }

}

void allocate_dmap(RigidBody* body)
{

  DistanceMap<float, cpu> map_(body->map_);

  cudaMalloc((void**)&(body->map_gpu_), sizeof(DistanceMap<float, gpu>));
  cudaCheckErrors("Allocate dmap");

  cudaMemcpy(body->map_gpu_, &map_, sizeof(DistanceMap<float, cpu>), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy distancemap class");

  body->map_gpu_->transferData(map_);

  gpu_map_test <<<1, 1>>>(body->map_gpu_);

  //dist_comp<<<1,1>>>(body->map_gpu_);

  cudaDeviceSynchronize();

  std::pair<Real, Vector3<Real> > result0 = body->map_->queryMap(VECTOR3(0.001,0,0));

  printf("map0: %f\n", result0.first);

}

void allocate_distancemaps(std::vector<RigidBody*> &rigidBodies, std::vector<DistanceMap<Real>* > &maps, std::vector<int> &bodyToMap)
{

  std::vector< DistanceMap<float,gpu>* > gpu_maps;

  for(auto &map : maps)
  {
    DistanceMap<float, cpu> map_(map);

    DistanceMap<float,gpu> *map_gpu;

    cudaMalloc((void**)&(map_gpu), sizeof(DistanceMap<float, gpu>));
    cudaCheckErrors("Allocate dmap");

    cudaMemcpy(map_gpu, &map_, sizeof(DistanceMap<float, cpu>), cudaMemcpyHostToDevice);
    cudaCheckErrors("copy distancemap class");

    map_gpu->transferData(map_);

    std::pair<Real, Vector3<Real> > result0 = map->queryMap(VECTOR3(0.001,0,0));

    test_dist_comp<<<1,1>>>(map_gpu, result0.first);
    cudaDeviceSynchronize();

    gpu_maps.push_back(map_gpu);
  }

  for (unsigned i=0; i < rigidBodies.size(); ++i)
  {
    RigidBody *body = rigidBodies[i];

    body->map_gpu_ = gpu_maps[bodyToMap[i]];
  }

  cudaDeviceSynchronize();

}

void transfer_distancemap(RigidBody *body, DistanceMap<float,cpu> *map)
{

  DistanceMap<float,cpu> &map_=*map;

  cudaMalloc((void**)&(body->map_gpu_), sizeof(DistanceMap<float,gpu>));
  cudaCheckErrors("Allocate dmap");

  cudaMemcpy(body->map_gpu_, &map_, sizeof(DistanceMap<float,cpu>), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy distancemap class");

  body->map_gpu_->transferData(map_);
  cudaDeviceSynchronize();

}

void copy_distancemap(DistanceMap<Real,cpu> *map)
{

  DMap map_;

  map_.dim_[0] = map->dim_[0];
  map_.dim_[1] = map->dim_[1];

  map_.cells_[0] = map->cells_[0];
  map_.cells_[1] = map->cells_[1];
  map_.cells_[2] = map->cells_[2];

  map_.cellSize_ = map->cellSize_; 

  Vector3<float> vmin, vmax;
  vmin.x = (float)map->boundingBox_.vertices_[0].x;
  vmin.y = (float)map->boundingBox_.vertices_[0].y;
  vmin.z = (float)map->boundingBox_.vertices_[0].z;

  vmax.x = (float)map->boundingBox_.vertices_[1].x;
  vmax.y = (float)map->boundingBox_.vertices_[1].y;
  vmax.z = (float)map->boundingBox_.vertices_[1].z;

  map_.bv_.init(vmin, vmax);

  cudaMalloc((void**)&d_map, sizeof(DMap));
  cudaCheckErrors("Allocate dmap");

  cudaMemcpy(d_map, &map_, sizeof(DMap), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy distancemap class");

  Vector3<float> *vertexCoords;
  Vector3<float> *normals;
  Vector3<float> *contactPoints;      

  float *distance_;

  int size = map->dim_[0] * map->dim_[1]; 

  map->outputInfo();

  vertexCoords = new Vector3<float>[size];
  normals = new Vector3<float>[size];
  contactPoints = new Vector3<float>[size];
  distance_ = new float[size];

  for (int i = 0; i < size; i++)
  {
    vertexCoords[i].x = (float)map->vertexCoords_[i].x;
    vertexCoords[i].y = (float)map->vertexCoords_[i].y;
    vertexCoords[i].z = (float)map->vertexCoords_[i].z;

    normals[i].x = (float)map->normals_[i].x;
    normals[i].y = (float)map->normals_[i].y;
    normals[i].z = (float)map->normals_[i].z;

    contactPoints[i].x = (float)map->contactPoints_[i].x;
    contactPoints[i].y = (float)map->contactPoints_[i].y;
    contactPoints[i].z = (float)map->contactPoints_[i].z;

    distance_[i] = (float)map->distance_[i];
  }

  cudaMalloc((void**)&d_vertexCoords, size * sizeof(vector3));
  cudaCheckErrors("Allocate vertices distancemap");

  cudaMemcpy(d_vertexCoords, vertexCoords, size * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices distance");

  cudaMemcpy(&d_map->vertices_ , &d_vertexCoords, sizeof(vector3*), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices distance");

  cudaMalloc((void**)&d_normals, size * sizeof(vector3));
  cudaCheckErrors("Allocate vertices normals");

  cudaMemcpy(d_normals, normals, size * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices normals");

  cudaMemcpy(&d_map->normals_ , &d_normals, sizeof(vector3*), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices normals");

  cudaMalloc((void**)&d_contactPoints, size * sizeof(vector3));
  cudaCheckErrors("Allocate vertices contactPoints");

  cudaMemcpy(d_contactPoints, contactPoints, size * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices contactPoints");

  cudaMemcpy(&d_map->contactPoints_ , &d_contactPoints, sizeof(vector3*), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices contactPoints");

  cudaMalloc((void**)&d_distance_map, size * sizeof(float));
  cudaCheckErrors("Allocate distance");

  cudaMemcpy(d_distance_map, distance_, size * sizeof(float), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy distance");

  cudaMemcpy(&d_map->distance_ , &d_distance_map, sizeof(float*), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy distance");

  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);
  test_distmap<<<(size+255)/256, 256 >>>(d_map, d_vertexCoords);
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float elapsed_time;
  cudaEventElapsedTime(&elapsed_time, start, stop);
  cudaDeviceSynchronize();
  printf("GPU distmap coll: %3.8f [ms]\n", elapsed_time);

  cudaDeviceSynchronize();

  //  std::pair<Real,Vector3<Real>> res = map->queryMap(map->vertexCoords_[0]+Vector3<Real>(0.1,0,0));
  //  std::cout << "query_cpu" << map->vertexCoords_[0] << std::endl;
  //  std::cout << "cp" << res.second << std::endl;
  //  std::cout << "dist_cpu" << res.first << std::endl;
  //  std::cout << "dist[100]" <<  map->distance_[100] << std::endl;
  //  exit(0);

  CPerfTimer timer;
  timer.Start();
  for (int i = 0; i < size; i++)
  {
    map->queryMap(map->vertexCoords_[i]);
  }

  std::cout << "Elapsed time gpu[ms]:" <<  timer.GetTime() * 1000.0 << std::endl;
  delete[] vertexCoords;
  delete[] normals;
  delete[] contactPoints;
  delete[] distance_;

}

void copy_mesh(Model3D *model){

  int nVertices = 0;

  g_model = model;

  vector3  *meshVertices;

  for (auto &mesh : model->meshes_)
  {
    nVertices += mesh.numVerts_;
    meshVertices = (vector3*)malloc(sizeof(vector3)*mesh.numVerts_);
    for (int i = 0; i < mesh.vertices_.Size(); i++)
    {
      meshVertices[i].x = (real)mesh.vertices_[i].x;
      meshVertices[i].y = (real)mesh.vertices_[i].y;
      meshVertices[i].z = (real)mesh.vertices_[i].z;
    }
  }

  printf("Number of triangles: %i\n",nVertices);
  g_triangles = nVertices;

  cudaMalloc((void**)&d_vertices, nVertices * sizeof(vector3));
  cudaCheckErrors("Allocate vertices");

  cudaMemcpy(d_vertices, meshVertices, nVertices * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("Copy vertices");
  cudaDeviceSynchronize();

  cudaMemcpyToSymbol(d_nVertices, &nVertices, sizeof(int));
  cudaCheckErrors("Copy number of vertices");

  free(meshVertices);

  cudaDeviceSynchronize();

}

void my_cuda_func(Model3D *model, UnstructuredGrid<Real, DTraits> &grid){

  int nTriangles = 0;
  int nVertices = 0;

  g_model = model;

  triangle *meshTriangles;
  vector3  *meshVertices;

  for (auto &mesh : model->meshes_)
  {
    nTriangles += mesh.numFaces_;
    meshTriangles=(triangle*)malloc(sizeof(triangle)*mesh.numFaces_);
    for (int i = 0; i < mesh.faces_.Size(); i++)
    {
      meshTriangles[i].idx0 = mesh.faces_[i][0];
      meshTriangles[i].idx1 = mesh.faces_[i][1];
      meshTriangles[i].idx2 = mesh.faces_[i][2];
    }

    nVertices += mesh.numVerts_;
    meshVertices = (vector3*)malloc(sizeof(vector3)*mesh.numVerts_);
    for (int i = 0; i < mesh.vertices_.Size(); i++)
    {
      meshVertices[i].x = (real)mesh.vertices_[i].x;
      meshVertices[i].y = (real)mesh.vertices_[i].y;
      meshVertices[i].z = (real)mesh.vertices_[i].z;
    }
  }

  model->meshes_[0].generateTriangleBoundingBoxes();

  AABB3f *boxes = new AABB3f[nTriangles];
  cudaMalloc((void**)&d_boundingBoxes, sizeof(AABB3f)* nTriangles);
  for (int i = 0; i < nTriangles; i++)
  {
    vector3 vmin, vmax;
    vmin.x = (real)model->meshes_[0].triangleAABBs_[i].vertices_[0].x;
    vmin.y = (real)model->meshes_[0].triangleAABBs_[i].vertices_[0].y;
    vmin.z = (real)model->meshes_[0].triangleAABBs_[i].vertices_[0].z;

    vmax.x = (real)model->meshes_[0].triangleAABBs_[i].vertices_[1].x;
    vmax.y = (real)model->meshes_[0].triangleAABBs_[i].vertices_[1].y;
    vmax.z = (real)model->meshes_[0].triangleAABBs_[i].vertices_[1].z;

    boxes[i].init(vmin, vmax);
  }

  cudaMemcpy(d_boundingBoxes, boxes, nTriangles * sizeof(AABB3f), cudaMemcpyHostToDevice);

  delete[] boxes;

  printf("Number of triangles: %i\n",nTriangles);
  g_triangles = nTriangles;
  cudaMalloc((void**)&d_triangles, nTriangles * sizeof(triangle));
  cudaCheckErrors("Allocate triangles");

  cudaMemcpy(d_triangles, meshTriangles, nTriangles * sizeof(triangle),cudaMemcpyHostToDevice);
  cudaCheckErrors("Copy triangles");

  cudaMemcpyToSymbol(d_nTriangles, &nTriangles, sizeof(int));
  cudaCheckErrors("Copy number of triangles");

  printf("CPU: Triangle[52].idx0 = %i \n", meshTriangles[52].idx0);

  cudaMalloc((void**)&d_vertices, nVertices * sizeof(vector3));
  cudaCheckErrors("Allocate vertices");

  cudaMalloc((void**)&d_inout, grid.nvt_ * sizeof(int));
  cudaCheckErrors("Allocate vertex traits");

  cudaMemcpy(d_vertices, meshVertices, nVertices * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("Copy vertices");
  cudaDeviceSynchronize();

  cudaMemcpyToSymbol(d_nVertices, &nVertices, sizeof(int));
  cudaCheckErrors("Copy number of vertices");

  printf("CPU: Number of vertices: %i\n", nVertices);
  printf("CPU: Vertex[52].y = %f \n", meshVertices[52].y);

  free(meshTriangles);

  free(meshVertices);

  cudaMalloc((void**)&d_vertices_grid, grid.nvt_ * sizeof(vector3));
  cudaCheckErrors("Allocate grid vertices");

  meshVertices = (vector3*)malloc(sizeof(vector3)*grid.nvt_);
  for (int i = 0; i < grid.nvt_; i++)
  {
    meshVertices[i].x = (real)grid.vertexCoords_[i].x;
    meshVertices[i].y = (real)grid.vertexCoords_[i].y;
    meshVertices[i].z = (real)grid.vertexCoords_[i].z;
  }

  cudaMemcpy(d_vertices_grid, meshVertices, grid.nvt_ * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("Copy grid vertices");
  cudaDeviceSynchronize();

  cudaMemcpyToSymbol(d_nVertices_grid, &grid.nvt_, sizeof(int));
  g_verticesGrid = grid.nvt_;
  free(meshVertices);

  cudaMalloc((void**)&d_distance, grid.nvt_ * sizeof(real));
  cudaCheckErrors("Allocation for distance array");

  cudaMemset(d_distance, 0, grid.nvt_ * sizeof(real));

}

void allocateNodes(std::list<int> *triangleIdx, AABB3f *boxes, int *pSize, int nNodes)
{

  cudaMalloc((void**)&d_nodes, nNodes * sizeof(BVHNode<float>));
  cudaCheckErrors("Allocate nodes");

  for (int i = 0; i < nNodes; i++)
  {
    cudaMemcpy(&d_nodes[i].nTriangles_, &pSize[i], sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(&(d_nodes[i].bv_), &boxes[i], sizeof(AABB3f), cudaMemcpyHostToDevice);
  }

  int **d_indices = new int*[nNodes];

  for (int i = 0; i < nNodes; i++)
  {
    cudaMalloc((void**)&d_indices[i], pSize[i] * sizeof(int));
    cudaMemcpy(&d_nodes[i].indices_, &d_indices[i], sizeof(int*), cudaMemcpyHostToDevice);
  }

  cudaDeviceSynchronize();

  int tsize = triangleIdx[0].size();

  printf("nodes = %i, psize[0] = %i %i \n", nNodes, pSize[0],tsize);
  int indices[10000];
  int j = 0;

  for (auto &idx : triangleIdx[0])
  {
    indices[j] = idx;
    j++;
  }
  printf("CPU: nodes[1].indices_[0] = %i \n", indices[0]);

  for (int i = 0; i < nNodes; i++)
  {

    int j = 0;
    for (auto &idx : triangleIdx[i])
    {
      indices[j] = idx;
      j++;
    }
    if (i==0)
      printf("CPU: nodes[1].indices_[0] = %i \n", indices[0]);

    cudaMemcpy(d_indices[i], indices, sizeof(int) * pSize[i], cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
  }

  printf("gpu triangles = %i \n", pSize[1]);
  printf("center = %f \n", boxes[1].center_.x);

}

void query_uniformgrid(RigidBody *body, UniformGrid<Real,ElementCell,VertexTraits<Real>> &grid)
{

  int size(body->map_->dim_[0] * body->map_->dim_[1]);

  int vx(grid.traits_.cells_[0]+1);
  int vxy(vx*vx*vx);

  cudaDeviceSynchronize();
  vector3 com(body->com_.x, body->com_.y, body->com_.z);

  Mat3 m(body->matTransform_);
  m.TransposeMatrix();

  Mat3f myMat;
  myMat.m_d00 = m.m_d00;
  myMat.m_d01 = m.m_d01;
  myMat.m_d02 = m.m_d02;

  myMat.m_d10 = m.m_d10;
  myMat.m_d11 = m.m_d11;
  myMat.m_d12 = m.m_d12;

  myMat.m_d20 = m.m_d20;
  myMat.m_d21 = m.m_d21;
  myMat.m_d22 = m.m_d22;

  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);

  dmap_kernel<<< (vxy+255)/256, 256 >>>(d_unigrid_gpu, body->map_gpu_, com, myMat);

  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);

  cudaDeviceSynchronize();

  float elapsed_time;
  cudaEventElapsedTime(&elapsed_time, start, stop);
  cudaDeviceSynchronize();

  float gpu_distmap = elapsed_time;

  int size2 = (grid.m_iDimension[0]+1) * (grid.m_iDimension[1]+1) * (grid.m_iDimension[2]+1);

  std::vector<int> fbmVertices(size2);
  int *dev_fbmVertices;

  cudaMalloc((void**)&(dev_fbmVertices), size2 * sizeof(int));

  copyFBM<<< (size2+255)/256, 256 >>>(d_unigrid_gpu,dev_fbmVertices,size2);

  cudaMemcpy(fbmVertices.data(),
      dev_fbmVertices,
      size2 * sizeof(int),
      cudaMemcpyDeviceToHost);

  cudaDeviceSynchronize();

  int inside(0);

  for(int i(0); i < size2; ++i)
  {
    if (fbmVertices[i])
    {
      inside++;
      grid.traits_.fbmVertices_[i] = fbmVertices[i];
    }
  }

  int inside_cpu(0);
  for (int i(0); i < size2; ++i)
  {     
    std::pair<Real, Vector3<Real>> res = body->map_->queryMap(grid.traits_.vertexCoords_[i]);
    if (res.first < 0.0) inside_cpu++;
  }

  test_inside(inside_cpu, inside);
  printf("> Elapsed time gpu distmap: %3.8f [ms].\n", elapsed_time);

  cudaFree(dev_fbmVertices);

}

void cleanGPU()
{

  cudaFree(d_triangles);
  cudaFree(d_vertices);
  cudaFree(d_vertices_grid);
  cudaFree(d_inout);
  cudaFree(d_distance);
  cudaFree(d_nodes);

}

