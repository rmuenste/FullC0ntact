#ifndef __difi__cuh__
#define __difi__cuh__

#define cudaCheckErrors(msg) cudaCheckError(msg,__FILE__, __LINE__)

void cudaCheckError(const char *message, const char *file, const int line)
{
  cudaError err = cudaGetLastError();
  if (cudaSuccess != err)
  {
    fprintf(stderr, "cudaCheckError() failed at %s:%i : %s User error message: %s\n",
      file, line, cudaGetErrorString(err), message);
    exit(-1);
  }

}

// Global variables

int g_triangles;
int g_verticesGrid;

C3DModel *g_model;

triangle *d_triangles;
vector3 *d_vertices;
vector3 *d_vertices_grid;

AABB3f *d_boundingBoxes;

int *d_inout;
real *d_distance;

#endif