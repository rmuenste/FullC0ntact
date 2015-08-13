#include <stdio.h>
#include <common.h>
#include <perftimer.h>
#include <cfloat>
#include <difi.cuh>
#include <aabb3.h>
#include <boundingvolumetree3.h>

#include "intersection.cuh"
#include "distance.cuh"
#include "unit_tests.cuh"



__device__ float machine_eps_flt() {
  typedef union {
    int i32;
    float f32;
  } flt_32;

  flt_32 s;

  s.f32 = 1.;
  s.i32++;
  return (s.f32 - 1.);
}

__global__ void test_eps()
{
  float eps = machine_eps_flt();
  printf("CUDA = %g, CPU = %g\n", eps, FLT_EPSILON);
}



__global__ void d_test_points(vector3 *vertices_grid, triangle *triangles, vector3 *vertices, int *traits)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

 
  //if( idx < d_nVertices_grid)
#ifdef TESTING
  if (idx == DEBUG_IVT)
#else
  if( idx < d_nVertices_grid)
#endif
  {
    //vector3 dir(0.9f,0.1f,0.2f);
    vector3 &query = vertices_grid[idx];
    vector3 dir(1.0f, 0.0f, 0.0f);
    int maxComp;
    if (fabs(query.x) > fabs(query.y))
      maxComp = 0;
    else
      maxComp = 1;

    if (fabs(query.m_dCoords[maxComp]) < fabs(query.z))
      maxComp = 2;

    if (query.m_dCoords[maxComp] < 0.0f)
    {
      dir = vector3(0.f, 0.f, 0.f);
      dir.m_dCoords[maxComp] = -1.0f;
    }
    else
    {
      dir = vector3(0.f, 0.f, 0.f);
      dir.m_dCoords[maxComp] = 1.0f;
    }

    //dir.normalize();
    int nIntersections = 0;
    int nTriangles = d_nTriangles;
    for(int i = 0; i < nTriangles; i++)
    {
      vector3 &v0 = vertices[triangles[i].idx0];
      vector3 &v1 = vertices[triangles[i].idx1];
      vector3 &v2 = vertices[triangles[i].idx2];
      if (intersection_tri(query, dir, v0, v1, v2,i))
      {
        nIntersections++;
#ifdef TESTING
        printf("Point [%f,%f,%f] hit with triangle %i \n",query.x,query.y,query.z,i);
#endif
      }
#ifdef TESTING
      else if (i == DEBUG_IDX)
      {
        printf("Point [%f,%f,%f] no hit with triangle %i \n", query.x, query.y, query.z, i);
      }

#endif
    }
    if(nIntersections%2!=0)
      traits[idx] = 1;
  }
}

void all_points_test(UnstructuredGrid<Real, DTraits> &grid)
{

  int *intersect = new int[grid.nvt_];
   
  cudaMemset(d_inout, 0, grid.nvt_ * sizeof(int));
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);
  d_test_points<<< (grid.nvt_ + 1023)/1024 , 1024 >>> (d_vertices_grid, d_triangles, d_vertices, d_inout);
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float elapsed_time;
  cudaEventElapsedTime(&elapsed_time, start, stop);
  cudaDeviceSynchronize();
  printf("GPU time event: %3.8f [ms]\n", elapsed_time);
  cudaMemcpy(intersect, d_inout, sizeof(int) * grid.nvt_, cudaMemcpyDeviceToHost);
  int id = 0;
  for(id=0; id < grid.nvt_; id++)
  {
    if (intersect[id])
    {
      grid.m_myTraits[id].iTag = 1;
    }
    else
    {
      grid.m_myTraits[id].iTag = 0;
    }
  }
  delete[] intersect;
}

__global__ void d_points_dist(vector3 *vertices_grid, triangle *triangles, vector3 *vertices, AABB3f *boxes, real *distance)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if (idx < d_nVertices_grid)
  {
    vector3 &query = vertices_grid[idx];
    int nTriangles = d_nTriangles;
    real min_dist = FLT_MAX;
    for (int i = 0; i < nTriangles; i++)
    {
      vector3 &v0 = vertices[triangles[i].idx0];
      vector3 &v1 = vertices[triangles[i].idx1];
      vector3 &v2 = vertices[triangles[i].idx2];
      if (boxes[i].minDistanceSqr(query) < min_dist)
      {
        real dist = distance_triangle<real>(query, v0, v1, v2);
        if (dist < min_dist)
        {
          min_dist = dist;
        }
      }
    }
    distance[idx] = min_dist;
  }
}

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

}

void my_cuda_func(C3DModel *model, UnstructuredGrid<Real, DTraits> &grid){
  
  int nTriangles = 0;
  int nVertices = 0;

  g_model = model;

  triangle *meshTriangles;
  vector3  *meshVertices;

  for (auto &mesh : model->m_vMeshes)
  {
    nTriangles += mesh.m_iNumFaces;
    meshTriangles=(triangle*)malloc(sizeof(triangle)*mesh.m_iNumFaces);
    for (int i = 0; i < mesh.m_pFaces.Size(); i++)
    {
      meshTriangles[i].idx0 = mesh.m_pFaces[i][0];
      meshTriangles[i].idx1 = mesh.m_pFaces[i][1];
      meshTriangles[i].idx2 = mesh.m_pFaces[i][2];
    }

    nVertices += mesh.m_iNumVerts;
    meshVertices = (vector3*)malloc(sizeof(vector3)*mesh.m_iNumVerts);
    for (int i = 0; i < mesh.m_pVertices.Size(); i++)
    {
      meshVertices[i].x = (real)mesh.m_pVertices[i].x;
      meshVertices[i].y = (real)mesh.m_pVertices[i].y;
      meshVertices[i].z = (real)mesh.m_pVertices[i].z;
    }
  }

  model->m_vMeshes[0].generateTriangleBoundingBoxes();

  AABB3f *boxes = new AABB3f[nTriangles];
  cudaMalloc((void**)&d_boundingBoxes, sizeof(AABB3f)* nTriangles);
  for (int i = 0; i < nTriangles; i++)
  {
    vector3 vmin, vmax;
    vmin.x = (real)model->m_vMeshes[0].triangleAABBs_[i].vertices_[0].x;
    vmin.y = (real)model->m_vMeshes[0].triangleAABBs_[i].vertices_[0].y;
    vmin.z = (real)model->m_vMeshes[0].triangleAABBs_[i].vertices_[0].z;

    vmax.x = (real)model->m_vMeshes[0].triangleAABBs_[i].vertices_[1].x;
    vmax.y = (real)model->m_vMeshes[0].triangleAABBs_[i].vertices_[1].y;
    vmax.z = (real)model->m_vMeshes[0].triangleAABBs_[i].vertices_[1].z;

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

__global__ void test_structure(BVHNode<float> *nodes, int *indices)
{
  printf("triangles = %i \n",nodes[1].nTriangles_);
  printf("center = %f \n", nodes[1].bv_.center_.x);

  printf("GPU: nodes[1].indices_[0] = %i \n", nodes[0].indices_[0]);
  printf("GPU: indices_[0] = %i \n", indices[0]);

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
  printf("nodes = %i, psize[0] = %i %i \n", nNodes, pSize[0],triangleIdx[0].size());
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

  test_structure << < 1, 1 >> >(d_nodes, d_indices[0]);

  printf("gpu triangles = %i \n", pSize[1]);
  printf("center = %f \n", boxes[1].center_.x);


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
