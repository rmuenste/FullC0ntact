#ifndef __common__h__
#define __common__h__
#include <3dmodel.h>
#include <unstructuredgrid.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <list>
using namespace i3d;

__constant__ int d_nVertices;

__constant__ int d_nTriangles;

__constant__ int d_nVertices_grid;

const int threadsPerBlock = 1024;

struct triangle
{
  unsigned idx0;
  unsigned idx1;
  unsigned idx2;
};

void triangle_test(UnstructuredGrid<Real, DTraits> &grid);

void my_cuda_func(Model3D *model, UnstructuredGrid<Real, DTraits> &grid);

void cleanGPU();

void single_triangle_test(UnstructuredGrid<Real, DTraits> &grid);

void single_point(UnstructuredGrid<Real, DTraits> &grid);

void all_points_test(UnstructuredGrid<Real, DTraits> &grid);

void all_points_dist(UnstructuredGrid<Real, DTraits> &grid);

void allocateNodes(std::list<int> *triangleIdx, AABB3f *boxes, int *pSize, int nNodes);

typedef float real;
typedef Vector3<float> vector3;

#endif
