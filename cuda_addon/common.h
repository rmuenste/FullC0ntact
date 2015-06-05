#ifndef __common__h__
#define __common__h__
#include <3dmodel.h>
#include <unstructuredgrid.h>

using namespace i3d;

struct triangle
{
  unsigned idx0;
  unsigned idx1;
  unsigned idx2;
};

void triangle_test(UnstructuredGrid<Real, DTraits> &grid);

void my_cuda_func(C3DModel *model, UnstructuredGrid<Real, DTraits> &grid);

void cleanGPU();

void single_triangle_test(UnstructuredGrid<Real, DTraits> &grid);

void single_point(UnstructuredGrid<Real, DTraits> &grid);

#endif
