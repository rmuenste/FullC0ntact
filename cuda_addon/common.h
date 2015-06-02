#ifndef __common__h__
#define __common__h__
#include <3dmodel.h>

using namespace i3d;

struct triangle
{
  unsigned idx0;
  unsigned idx1;
  unsigned idx2;
};

void my_cuda_func(C3DModel *model);

void cleanGPU();

#endif