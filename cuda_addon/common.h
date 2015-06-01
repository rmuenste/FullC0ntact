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

struct vector3
{
  union {
    struct {
      float x, y, z;
    };
    float entries[3];
  };
};

void my_cuda_func(C3DModel *model);

void cleanGPU();

#endif