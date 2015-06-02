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

struct vector3
{
  union {
    struct {
      float x, y, z;
    };
    float entries[3];
  };

  __device__ __host__ vector3() : x(0), y(0), z(0)
  {

  }

  __device__ __host__ vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z)
  {

  }

  __device__ __host__ vector3(const vector3 &copy) : x(copy.x), y(copy.y), z(copy.z)
  {

  }

  __device__ __host__ inline float sqrLength()
  {
    return (x*x + y*y + z*z);
  }

  __device__ __host__ vector3 operator+(const vector3 &rhs)
  {
    return vector3(x + rhs.x, y + rhs.y, z + rhs.z);
  }

  __device__ __host__ vector3 operator-(const vector3 &rhs) const
  {
    return vector3(x - rhs.x, y - rhs.y, z - rhs.z);
  }

  __device__ __host__ float operator*(const vector3 &rhs) const
  {
    return x*rhs.x + y*rhs.y + z*rhs.z;
  }

  __device__ __host__ vector3 operator*(float s)
  {
    return vector3(x*s, y*s, z*s);
  }

  __device__ __host__ inline static vector3 cross(vector3 vVector1, vector3 vVector2)
  {
    vector3 vCross;

    vCross.x = ((vVector1.y * vVector2.z) - (vVector1.z * vVector2.y));

    vCross.y = ((vVector1.z * vVector2.x) - (vVector1.x * vVector2.z));

    vCross.z = ((vVector1.x * vVector2.y) - (vVector1.y * vVector2.x));

    return vCross;
  }

};

triangle *d_triangles;
vector3 *d_vertices;

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
  float fDdN = dir * kNormal;
  float fSign;
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

  float fDdQxE2 = (dir * vector3::cross(kDiff, kEdge2)) * fSign;

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

__device__ bool intersection_tri(const vector3 &orig, const vector3 &dir, const vector3 &v0, const vector3 &v1, const vector3 &v2)
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

  float fDdQxE2 = (dir * vector3::cross(kDiff, kEdge2)) * fSign;

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

__global__ void my_kernel(triangle *triangles, vector3 *vertices){
  printf("Hello!\n");

  printf("Number of triangles on GPU: %i \n", d_nTriangles);
  printf("GPU Triangle[52].idx0 = %i \n", triangles[52].idx0);

  printf("Number of vertices on GPU: %i \n", d_nVertices);
  printf("GPU vertices[52].y = %f \n", vertices[52].y);
}

__global__ void vec_add_test(vector3 *vertices)
{

  vector3 sum = vertices[0] + vertices[1];
  printf("GPU: [%f,%f,%f] \n", sum.x, sum.y, sum.z);

}

__global__ void triangle_intersection_test()
{

  vector3 orig(0.0f, 0.0f, 0.0f);
  vector3 dir(0.9, 0.8, 0.02);

  vector3 v0(0.474477, 0.268391, 0.162952);
  vector3 v1(0.393, 0.263599, -0.004793);
  vector3 v2(0.297147, 0.335489, 0.014378);

  if (intersection_tri(orig, dir, v0, v1, v2))
  {
    printf("GPU: intersection \n");
  }



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

  //determine ray direction
  //Vector3<T> dir(0.9, 0.8, 0.02);/// = vQuery - pNode->m_BV.GetCenter();

  //CRay3(const Vector3<T> &vOrig, const Vector3<T> &vDir);
  //Ray3<T> ray(vQuery, dir);

  my_kernel <<<1, 1 >>>(d_triangles, d_vertices);
  cudaDeviceSynchronize();

  vec_add_test <<< 1, 1 >>>(d_vertices);
  cudaDeviceSynchronize();

  std::cout << "CPU: " << model->m_vMeshes[0].m_pVertices[0] + model->m_vMeshes[0].m_pVertices[1] << std::endl;

  triangle_intersection_test <<< 1, 1 >>>();
  cudaDeviceSynchronize();

}

void cleanGPU()
{
  cudaFree(d_triangles);
}