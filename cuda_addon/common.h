#ifndef __common__h__
#define __common__h__
#include <3dmodel.h>
#include <unstructuredgrid.h>
#include <cuda.h>
#include <cuda_runtime.h>

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

//template <typename T>
//struct v3
//{
//  union {
//    struct {
//      T x, y, z;
//    };
//    T entries[3];
//  };
//
//  __device__ __host__ v3() : x(0), y(0), z(0)
//  {
//
//  }
//
//  __device__ __host__ v3(float _x, float _y, float _z) : x(_x), y(_y), z(_z)
//  {
//
//  }
//
//  __device__ __host__ v3(const v3 &copy) : x(copy.x), y(copy.y), z(copy.z)
//  {
//
//  }
//
//  __device__ __host__ inline float sqrLength()
//  {
//    return (x*x + y*y + z*z);
//  }
//
//  __device__ __host__ v3 operator+(const v3 &rhs) const
//  {
//    return v3(x + rhs.x, y + rhs.y, z + rhs.z);
//  }
//
//  __device__ __host__ v3 operator-(const v3 &rhs) const
//  {
//    return v3(x - rhs.x, y - rhs.y, z - rhs.z);
//  }
//
//  __device__ __host__ T operator*(const v3 &rhs) const
//  {
//    return x*rhs.x + y*rhs.y + z*rhs.z;
//  }
//
//  __device__ __host__ v3 operator*(T s) const
//  {
//    return v3(x*s, y*s, z*s);
//  }
//
//  __device__ __host__ inline static v3 cross(v3 vVector1, v3 vVector2)
//  {
//    v3 vCross;
//
//    vCross.x = ((vVector1.y * vVector2.z) - (vVector1.z * vVector2.y));
//
//    vCross.y = ((vVector1.z * vVector2.x) - (vVector1.x * vVector2.z));
//
//    vCross.z = ((vVector1.x * vVector2.y) - (vVector1.y * vVector2.x));
//
//    return vCross;
//  }
//
//};

void triangle_test(UnstructuredGrid<Real, DTraits> &grid);

void my_cuda_func(C3DModel *model, UnstructuredGrid<Real, DTraits> &grid);

void cleanGPU();

void single_triangle_test(UnstructuredGrid<Real, DTraits> &grid);

void single_point(UnstructuredGrid<Real, DTraits> &grid);

void all_points_test(UnstructuredGrid<Real, DTraits> &grid);

void all_points_dist(UnstructuredGrid<Real, DTraits> &grid);

typedef float real;
typedef Vector3<float> vector3;

#endif
