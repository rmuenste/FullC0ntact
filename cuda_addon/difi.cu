#include <stdio.h>
#include <common.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <perftimer.h>
#include <cfloat>

int g_triangles;

C3DModel *g_model;
int g_verticesGrid;
__constant__ int d_nVertices;

__constant__ int d_nTriangles;

__constant__ int d_nVertices_grid;

const int threadsPerBlock = 1024;

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

template <typename T>
struct v3
{
  union {
    struct {
      T x, y, z;
    };
    T entries[3];
  };

  __device__ __host__ v3() : x(0), y(0), z(0)
  {

  }

  __device__ __host__ v3(float _x, float _y, float _z) : x(_x), y(_y), z(_z)
  {

  }

  __device__ __host__ v3(const v3 &copy) : x(copy.x), y(copy.y), z(copy.z)
  {

  }

  __device__ __host__ inline float sqrLength()
  {
    return (x*x + y*y + z*z);
  }

  __device__ __host__ v3 operator+(const v3 &rhs) const
  {
    return v3(x + rhs.x, y + rhs.y, z + rhs.z);
  }

  __device__ __host__ v3 operator-(const v3 &rhs) const
  {
    return v3(x - rhs.x, y - rhs.y, z - rhs.z);
  }

  __device__ __host__ T operator*(const v3 &rhs) const
  {
    return x*rhs.x + y*rhs.y + z*rhs.z;
  }

  __device__ __host__ v3 operator*(T s) const
  {
    return v3(x*s, y*s, z*s);
  }

  __device__ __host__ inline static v3 cross(v3 vVector1, v3 vVector2)
  {
    v3 vCross;

    vCross.x = ((vVector1.y * vVector2.z) - (vVector1.z * vVector2.y));

    vCross.y = ((vVector1.z * vVector2.x) - (vVector1.x * vVector2.z));

    vCross.z = ((vVector1.x * vVector2.y) - (vVector1.y * vVector2.x));

    return vCross;
  }

};

typedef float real;
typedef v3<real> vector3;


triangle *d_triangles;
vector3 *d_vertices;
vector3 *d_vertices_grid;
int *d_inout;

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

__device__ float ComputeDistance(const vector3 &query, triangle *tri, vector3 *vertices)
{
  float da, db, dc, dd, de,ds,dt;
  float ddenom, dnum;

  vector3 e0, e1, vClosestPoint;

  vector3 &v0 = vertices[tri->idx0];
  vector3 &v1 = vertices[tri->idx1];
  vector3 &v2 = vertices[tri->idx2];

  e0 = v1 - v0;
  e1 = v2 - v0;

  da = e0 * e0;
  db = e0 * e1;
  dc = e1 * e1;
  dd = e0 * (v0 - query);
  de = e1 * (v0 - query);

  ddenom = da*dc - db*db;
  ds = db*de - dc*dd;
  dt = db*dd - da*de;

  if (ds + dt <= ddenom)
  {
    if (ds < 0)
    {
      if (dt < 0)
      {
        if (de < 0.0)
        {
          ds = (dd >= 0 ? 0 : (-dd >= da ? 1 : dd / da));
          dt = 0;
        }
        else
        {
          ds = 0;
          dt = (de >= 0 ? 0 : (-de >= dc ? 1 : -de / dc));
        }
      }
      //Region 3
      else
      {
        ds = 0;
        dt = (de >= 0 ? 0 : (-de >= dc ? 1 : -de / dc));
      }
    }
    //Region 5
    else if (dt < 0)
    {
      ds = (dd >= 0 ? 0 : (-dd >= da ? 1 : -dd / da));
      dt = 0;
    }
    //Region 0
    else
    {
      float invDenom = float(1.0) / ddenom;
      ds *= invDenom;
      dt *= invDenom;
    }
  }
  else
  {
    //Region 2
    if (ds < 0)
    {
      float tmp0 = db + dd;
      float tmp1 = dc + de;
      if (tmp1 > tmp0)
      {
        dnum = tmp1 - tmp0;
        ddenom = da - 2 * db + dc;
        ds = (dnum >= ddenom ? 1 : dnum / ddenom);
        dt = 1 - ds;
      }
      else
      {
        ds = 0;
        dt = (tmp1 <= 0 ? 1 : (de >= 0 ? 0 : -de / dc));
      }
    }
    //Region 6
    else if (dt < 0)
    {
      float tmp0 = -da - dd;
      float tmp1 = db + de;
      float tmp2 = db + dd;
      float tmp3 = dc + de;
      if (tmp1 > tmp0)
      {
        dnum = tmp3 - tmp2;
        ddenom = da - 2 * db + dc;
        ds = (dnum >= ddenom ? 1 : dnum / ddenom);
        dt = 1 - ds;
      }
      else
      {
        ds = (dd >= 0 ? 0 : (-dd >= da ? 1 : dd / da));
        dt = 0;
      }
    }
    // Region 1
    else
    {
      dnum = (dc + de - db - dd);
      if (dnum <= 0)
      {
        ds = 0.0;
      }
      else
      {
        ddenom = da - 2 * db + dc;
        ds = (dnum >= ddenom ? 1.0 : dnum / ddenom);
      }
      dt = 1 - ds;
    }
  }


  vector3 closestPoint = v0 + e0*ds + e1*dt;

  return (query - closestPoint).sqrLength();

}//end ComputeDistance

__device__ bool intersection(const vector3 &orig, const vector3 &dir, triangle *tri, vector3* vertices)
{

  // compute the offset origin, edges, and normal
  vector3 &v0 = vertices[tri->idx0];
  vector3 &v1 = vertices[tri->idx1];
  vector3 &v2 = vertices[tri->idx2];

  vector3 kDiff = orig - v0;
  vector3 kEdge1 = v1 - v0;
  vector3 kEdge2 = v2 - v0;
  vector3 kNormal = vector3::cross(kEdge1, kEdge2);

  // Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = ray direction,
  // E1 = kEdge1, E2 = kEdge2, N = Cross(E1,E2)) by
  //   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
  //   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
  //   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
  real fDdN = dir * kNormal;
  real fSign;
  if (fDdN > 0.0000005)
  {
    fSign = 1.0;
  }
  else if (fDdN < 0.0000005)
  {
    fSign = -1.0f;
    fDdN = -fDdN;
  }
  else
  {
    // Ray and triangle are parallel, call it a "no intersection"
    // even if the ray does intersect.
    return false;
  }

  real fDdQxE2 = (dir * vector3::cross(kDiff, kEdge2)) * fSign;

  if (fDdQxE2 >= 0.0)
  {
    Real fDdE1xQ = (dir * vector3::cross(kEdge1, kDiff)) * fSign;
    if (fDdE1xQ >= 0.0)
    {
      if (fDdQxE2 + fDdE1xQ <= fDdN)
      {
        // line intersects triangle, check if ray does
        Real fQdN = (kDiff * kNormal) * -fSign;
        if (fQdN >= 0.0)
        {
          // ray intersects triangle
          return true;
        }
        // else: t < 0, no intersection
      }
      // else: b1+b2 > 1, no intersection
    }
    // else: b2 < 0, no intersection
  }
  // else: b1 < 0, no intersection

  return false;

}//end Intersection


//#define TESTING
//#define DEBUG_IDX 917
//#define DEBUG_IVT 339
__device__ bool intersection_tri(const vector3 &orig, const vector3 &dir, const vector3 &v0, const vector3 &v1, const vector3 &v2, int idx)
{

  vector3 kDiff = orig - v0;
  vector3 kEdge1 = v1 - v0;
  vector3 kEdge2 = v2 - v0;
  vector3 kNormal = vector3::cross(kEdge1, kEdge2);

  // Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = ray direction,
  // E1 = kEdge1, E2 = kEdge2, N = Cross(E1,E2)) by
  //   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
  //   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
  //   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
  float fDdN = dir * kNormal;
  float fSign;
  if (fDdN > FLT_EPSILON)
  {
    fSign = 1.0;
  }
  else if (fDdN < -FLT_EPSILON)
  {
    fSign = -1.0f;
    fDdN = -fDdN;
  }
  else
  {
    // Ray and triangle are parallel, call it a "no intersection"
    // even if the ray does intersect.
#ifdef TESTING
    if (idx == DEBUG_IDX)
      printf("parallel\n");
#endif
    return false;
  }

  float fDdQxE2 = (dir * vector3::cross(kDiff, kEdge2)) * fSign;
              
  if (fDdQxE2 > -FLT_EPSILON)// FLT_EPSILON) //FLT_EPSILON
  {
    Real fDdE1xQ = (dir * vector3::cross(kEdge1, kDiff)) * fSign;
    if (fDdE1xQ >= 0.0f)
    {
      if (fDdQxE2 + fDdE1xQ <= fDdN + 0.000001f)
      {
        // line intersects triangle, check if ray does
        Real fQdN = (kDiff * kNormal) * -fSign;
        if (fQdN >= 0.0f)
        {
          // ray intersects triangle
          return true;
        }
        // else: t < 0, no intersection
#ifdef TESTING
        else
        {
          if (idx == DEBUG_IDX)
            printf("t < 0\n");
        }
#endif
      }
      // else: b1+b2 > 1, no intersection
#ifdef TESTING
      else
      {
        if (idx == DEBUG_IDX)
        {
          printf("b1+b2 > 1\n");
          printf("%f + %f <= %f\n", fDdQxE2, fDdE1xQ, fDdN);
        }
      }
#endif
    }
    // else: b2 < 0, no intersection
#ifdef TESTING
    else
    {
      if (idx == DEBUG_IDX)
        printf("b2 < 0\n");
    }
#endif
  }
  // else: b1 < 0, no intersection
#ifdef TESTING
  else
  {
    if (idx == DEBUG_IDX)
    {
      printf("b1 < 0\n");
      printf("b1 = %6.18f\n", fDdQxE2);
    }
  }
#endif
  return false;

}//end Intersection

#include "unit_tests.cu"

__global__ void test_all_triangles(vector3 query, triangle *triangles, vector3* vertices, int *nintersections)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  //Vector3<float> dir_(0.1, 0.1, 1.0);
  //dir_.Normalize();
  //vector3 dir(dir_.x,dir_.y,dir_.z);
  vector3 dir(1.0, 0, 0.0);

  nintersections[idx]=0;
  if(idx < d_nTriangles)
  {
    vector3 &v0 = vertices[triangles[idx].idx0];
    vector3 &v1 = vertices[triangles[idx].idx1];
    vector3 &v2 = vertices[triangles[idx].idx2];
    if (intersection_tri(query, dir, v0, v1, v2,idx))
    {
      nintersections[idx]++;
    }
    //printf("threadIdx = %i, intersections = %i \n",idx, nintersections[idx]);
  }

}

void single_point(UnstructuredGrid<Real, DTraits> &grid)
{

  int *d_intersection;
  int intersection[threadsPerBlock];

  cudaMalloc((void**)&d_intersection, sizeof(int)*threadsPerBlock);
  int j = 0;
  int id = j;
  //VECTOR3 vQuery = VECTOR3();// grid.vertexCoords_[j];
  //VECTOR3 vQuery(-1.0, 0.25, 0.0);
  //VECTOR3 vQuery(-1.0, -0.34375, 0.34375);
  //VECTOR3 vQuery(-1.0, 0.125, 0.09375);
#ifdef DEBUG_IVT
  VECTOR3 vQuery = grid.vertexCoords_[DEBUG_IVT];
#else
  VECTOR3 vQuery(0,0,0);
#endif
  cudaMemset(&d_intersection, 0, sizeof(int)*threadsPerBlock);
  int intersections = 0;
  vector3 query(vQuery.x, vQuery.y, vQuery.z);
  //if (!g_model->GetBox().isPointInside(vQuery))
  //{
  //  continue;
  //}
  test_all_triangles << <1, threadsPerBlock >> > (query, d_triangles, d_vertices, d_intersection);
  cudaMemcpy(&intersection, d_intersection, sizeof(int)*threadsPerBlock, cudaMemcpyDeviceToHost);
  for (int i = 0; i < threadsPerBlock; i++)
  {
    intersections += intersection[i];
    if (intersection[i])
      std::cout << "GPU Intersection with " << i << std::endl;
  }
  std::cout << "nIntersections: " << intersections << std::endl;
  if (intersections % 2 != 0)
  {
    grid.m_myTraits[id].iTag = 1;
  }
  else
  {
    grid.m_myTraits[id].iTag = 0;
  }

}

__global__ void d_test_points(vector3 *vertices_grid, triangle *triangles, vector3 *vertices, int *traits)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;
  int j=0;
 
  if( idx < d_nVertices_grid)
  {
    vector3 dir(1.0f,0.0f,0.0f);
    vector3 &query = vertices_grid[idx];
    int nIntersections = 0;
    int nTriangles = 968;
    for(int i = 0; i < nTriangles; i++)
    {
      vector3 &v0 = vertices[triangles[i].idx0];
      vector3 &v1 = vertices[triangles[i].idx1];
      vector3 &v2 = vertices[triangles[i].idx2];
      if (intersection_tri(query, dir, v0, v1, v2,j))
      {
        nIntersections++;
//        printf("Point [%f,%f,%f] hit with triangle %i \n",query.x,query.y,query.z,i);
      }
    }
    if(nIntersections%2!=0)
      traits[idx] = 1;
  }
}

void all_points_test(UnstructuredGrid<Real, DTraits> &grid)
{

  int *intersect = new int[grid.nvt_];
   
  cudaMemset(d_inout, 0, grid.nvt_ * sizeof(int));
  CPerfTimer timer;
  timer.Start();
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
  double dt_gpu = timer.GetTime();
  //std::cout << "nIntersections: " << nIntersections << std::endl;
  //std::cout << "GPU time: " << dt_gpu << std::endl;
  printf("GPU time: %3.8f ms\n", dt_gpu);
  printf("GPU time event: %3.8f ms\n", elapsed_time);
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

void triangle_test(UnstructuredGrid<Real, DTraits> &grid)
{

  int *d_intersection;
  int intersection[threadsPerBlock];
  
  cudaMalloc((void**)&d_intersection,sizeof(int)*threadsPerBlock);
  CPerfTimer timer;
  timer.Start();
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);
  for (int j=0; j < grid.nvt_; j++)
  {
    int id = j; 
    VECTOR3 vQuery = grid.vertexCoords_[j];
    cudaMemset(&d_intersection,0, sizeof(int)*threadsPerBlock); 
    int intersections = 0;
    vector3 query(vQuery.x, vQuery.y, vQuery.z);
    if (!g_model->GetBox().isPointInside(vQuery))
    {
      continue;
    }

    test_all_triangles<<<1,threadsPerBlock>>> (query, d_triangles, d_vertices, d_intersection);
    cudaMemcpy(&intersection, d_intersection, sizeof(int)*threadsPerBlock, cudaMemcpyDeviceToHost);

    for(int i=0; i < threadsPerBlock; i++)
    {
      intersections+=intersection[i];
    }
//    if (intersections%2!=0)
//    {
//      grid.m_mytraits[id].itag = 1;
//    }
//    else
//    {
//      grid.m_mytraits[id].itag = 0;
//    }

  }
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float elapsed_time;
  cudaEventElapsedTime(&elapsed_time, start, stop);
  cudaDeviceSynchronize();
  double dt_gpu = timer.GetTime();
  //std::cout << "nIntersections: " << nIntersections << std::endl;
  //std::cout << "GPU time: " << dt_gpu << std::endl;
  printf("GPU time: %3.8f ms\n", dt_gpu);
  printf("GPU time event: %3.8f ms\n", elapsed_time);


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
      meshVertices[i].x = mesh.m_pVertices[i].x;
      meshVertices[i].y = mesh.m_pVertices[i].y;
      meshVertices[i].z = mesh.m_pVertices[i].z;
    }
  }

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
    meshVertices[i].x = grid.vertexCoords_[i].x;
    meshVertices[i].y = grid.vertexCoords_[i].y;
    meshVertices[i].z = grid.vertexCoords_[i].z;
  }

  cudaMemcpy(d_vertices_grid, meshVertices, grid.nvt_ * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("Copy grid vertices");
  cudaDeviceSynchronize();

  cudaMemcpyToSymbol(d_nVertices_grid, &grid.nvt_, sizeof(int));
  g_verticesGrid = grid.nvt_;
  free(meshVertices);

}

void cleanGPU()
{
  cudaFree(d_triangles);
}

