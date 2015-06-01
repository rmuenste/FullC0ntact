#include <stdio.h>
#include <common.h>
#include <cuda.h>
#include <cuda_runtime.h>

__device__ __constant__ int d_nTriangles;
__device__ __constant__ int d_nVertices;

#define cudaCheckErrors(msg) cudaCheckError(msg,__FILE__, __LINE__)

void cudaCheckError(const char *message, const char *file, const int line)
{
  cudaError err = cudaGetLastError();
  if (cudaSuccess != err)
  {
    fprintf(stderr, "cudaCheckError() failed at %s:%i : %s Error message: %s\n",
      file, line, cudaGetErrorString(err), message);
    exit(-1);
  }

}

triangle *d_triangles;
vector3 *d_vertices;

__global__ void my_kernel(triangle *triangles, vector3 *vertices){
  printf("Hello!\n");

  printf("Number of triangles on GPU: %i \n", d_nTriangles);
  printf("GPU Triangle[52].idx0 = %i \n", triangles[52].idx0);

  printf("Number of vertices on GPU: %i \n", d_nVertices);
  printf("GPU vertices[52].y = %f \n", vertices[52].y);
}

void my_cuda_func(C3DModel *model){
  
  int nTriangles = 0;
  int nVertices = 0;
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
      meshVertices[i].x = mesh.m_pVertices[i].x;
      meshVertices[i].y = mesh.m_pVertices[i].y;
      meshVertices[i].z = mesh.m_pVertices[i].z;

    }

  }

  printf("Number of triangles: %i\n",nTriangles);

  cudaMalloc((void**)&d_triangles, nTriangles * sizeof(triangle));
  cudaCheckErrors("Allocate triangles");

  cudaMemcpy(d_triangles, meshTriangles, nTriangles * sizeof(triangle),cudaMemcpyHostToDevice);
  cudaCheckErrors("Copy triangles");

  cudaMemcpyToSymbol(d_nTriangles, &nTriangles, sizeof(int*));
  cudaCheckErrors("Copy number of triangles");

  printf("CPU: Triangle[52].idx0 = %i \n", meshTriangles[52].idx0);

  cudaMalloc((void**)&d_vertices, nVertices * sizeof(vector3));
  cudaCheckErrors("Allocate vertices");

  cudaMemcpy(d_vertices, meshVertices, nVertices * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("Copy vertices");

  cudaMemcpyToSymbol(d_nVertices, &nVertices, sizeof(int*));
  cudaCheckErrors("Copy number of vertices");

  printf("CPU: Number of vertices: %i\n", nVertices);
  printf("CPU: Vertex[52].y = %f \n", meshVertices[52].y);

  free(meshTriangles);

  free(meshVertices);

  my_kernel <<<1, 1 >>>(d_triangles, d_vertices);
  cudaDeviceSynchronize();
}

void cleanGPU()
{
  cudaFree(d_triangles);
}