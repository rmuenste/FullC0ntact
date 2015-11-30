#include <stdio.h>
#include <common.h>
#include <perftimer.h>
#include <cfloat>
#include <difi.cuh>
#include <aabb3.h>
#include <boundingvolumetree3.h>

int g_triangles;
int g_verticesGrid;

Model3D *g_model;

triangle *d_triangles;
vector3 *d_vertices;
vector3 *d_vertices_grid;

AABB3f *d_boundingBoxes;

vector3 *d_vertexCoords;
vector3 *d_normals;
vector3 *d_contactPoints;      
  
float *d_distance_map;

int *d_inout;
real *d_distance;

#include "bvh.cuh"
#include "intersection.cuh"
#include "distance.cuh"
#include "unit_tests.cuh"

namespace i3d {
  template<typename T>
    class DistanceMap<T,gpu>
    {
      public:

        Vector3<T> *vertexCoords_;
        Vector3<T> *normals_;
        Vector3<T> *contactPoints_;      

        T *distance_;

        int *stateFBM_;

        AABB3<T> boundingBox_;

        int cells_[3];

        int dim_[2];

        // cell size
        T cellSize_;  

        __device__ __host__ 
          //ClosestPoint to vertex -> easily compute normal
          T trilinearInterpolateDistance(const Vector3<T> &vQuery, int indices[8]);

        __device__ __host__
          Vector3<T> trilinearInterpolateCP(const Vector3<T> &vQuery, int indices[8]);

        __device__ __host__
          void vertexIndices(int icellx,int icelly, int icellz, int indices[8]);
    };

  template<typename T>
    T DistanceMap<T,gpu>::trilinearInterpolateDistance(const Vector3<T> &vQuery, int indices[8])
    {
      //trilinear interpolation of distance
      T x_d= (vQuery.x - vertexCoords_[indices[0]].x)/(vertexCoords_[indices[1]].x - vertexCoords_[indices[0]].x);
      T y_d= (vQuery.y - vertexCoords_[indices[0]].y)/(vertexCoords_[indices[2]].y - vertexCoords_[indices[0]].y);
      T z_d= (vQuery.z - vertexCoords_[indices[0]].z)/(vertexCoords_[indices[4]].z - vertexCoords_[indices[0]].z);  

      T c00 = distance_[indices[0]] * (1.0 - x_d) + distance_[indices[1]] * x_d;

      T c10 = distance_[indices[3]] * (1.0 - x_d) + distance_[indices[2]] * x_d;

      T c01 = distance_[indices[4]] * (1.0 - x_d) + distance_[indices[5]] * x_d;

      T c11 = distance_[indices[7]] * (1.0 - x_d) + distance_[indices[6]] * x_d;      

      //next step
      T c0 = c00*(1.0 - y_d) + c10 * y_d;

      T c1 = c01*(1.0 - y_d) + c11 * y_d;  

      //final step
      T c = c0*(1.0 - z_d) + c1 * z_d;  

      return c;
    }

  template<typename T>
    Vector3<T> DistanceMap<T,gpu>::trilinearInterpolateCP(const Vector3<T> &vQuery, int indices[8])
    {
      //trilinear interpolation of distance
      T x_d= (vQuery.x - vertexCoords_[indices[0]].x)/(vertexCoords_[indices[1]].x - vertexCoords_[indices[0]].x);
      T y_d= (vQuery.y - vertexCoords_[indices[0]].y)/(vertexCoords_[indices[2]].y - vertexCoords_[indices[0]].y);
      T z_d= (vQuery.z - vertexCoords_[indices[0]].z)/(vertexCoords_[indices[4]].z - vertexCoords_[indices[0]].z);  

      Vector3<T> c00 = contactPoints_[indices[0]] * (1.0 - x_d) + contactPoints_[indices[1]] * x_d;

      Vector3<T> c10 = contactPoints_[indices[3]] * (1.0 - x_d) + contactPoints_[indices[2]] * x_d;

      Vector3<T> c01 = contactPoints_[indices[4]] * (1.0 - x_d) + contactPoints_[indices[5]] * x_d;

      Vector3<T> c11 = contactPoints_[indices[7]] * (1.0 - x_d) + contactPoints_[indices[6]] * x_d;      

      //next step
      Vector3<T> c0 = c00*(1.0 - y_d) + c10 * y_d;

      Vector3<T> c1 = c01*(1.0 - y_d) + c11 * y_d;  

      //final step
      Vector3<T> c = c0*(1.0 - z_d) + c1 * z_d;  

      return c;
    }

  template<typename T>
    void DistanceMap<T,gpu>::vertexIndices(int icellx,int icelly, int icellz, int indices[8])
    {

      int baseIndex=icellz*dim_[1]+icelly*dim_[0]+icellx; 

      indices[0]=baseIndex;         //xmin,ymin,zmin
      indices[1]=baseIndex+1;       //xmax,ymin,zmin

      indices[2]=baseIndex+dim_[0]+1; //xmax,ymax,zmin
      indices[3]=baseIndex+dim_[0];   //xmin,ymax,zmin


      indices[4]=baseIndex+dim_[1];  
      indices[5]=baseIndex+dim_[1]+1;  

      indices[6]=baseIndex+dim_[0]+dim_[1]+1;  
      indices[7]=baseIndex+dim_[0]+dim_[1];
    }
}

BVHNode<float> *d_nodes;
DMap *d_map;
DistanceMap<float,gpu> *d_map_gpu;

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

__global__ void hello_kernel()
{
  printf("Hello\n");
}

void hello_launcher()
{
  hello_kernel<<<1,1>>>();
  cudaDeviceSynchronize();
}

void all_points_dist(UnstructuredGrid<Real, DTraits> &grid)
{

  hello_kernel<<<1,1>>>();
  cudaDeviceSynchronize();
  exit(0);

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
  hello_kernel<<<1,1>>>();
  cudaDeviceSynchronize();

}

__global__ void test_distmap(DMap *map, vector3 *vertices)
{

  //  printf("cells = %i %i %i\n",map->cells_[0],map->cells_[1],map->cells_[2]);
  //  printf("dim = %i %i \n",map->dim_[0],map->dim_[1]);
  //  

  //  printf("vertex = %f %f %f\n", map->vertices_[1].x, map->vertices_[1].y, map->vertices_[1].z);
  //
  //  printf("normals = %f %f %f\n", map->normals_[1].x, map->normals_[1].y, map->normals_[1].z);
  //
  //  printf("contactPoints = %f %f %f\n", map->contactPoints_[1].x, map->contactPoints_[1].y, map->contactPoints_[1].z);
  //
  //  printf("distance_ = %f \n", map->distance_[1]);

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(idx < map->dim_[0]*map->dim_[1])
  {

    //    printf("center = %f %f %f\n", map->bv_.center_.x, map->bv_.center_.y, map->bv_.center_.z);
    //    printf("extends = %f %f %f\n", map->bv_.extents_[0], map->bv_.extents_[1], map->bv_.extents_[2]);
    vector3 query = vertices[idx];
    query += vector3(0.1,0,0); 
    vector3 cp(0,0,0);
    float dist=0;
    map->queryMap(query,dist,cp);

    //    if(blockIdx.x==0 && threadIdx.x==0)
    //    {
    //      map->queryMap(query,dist,cp);
    //      printf("dist_gpu = %f\n", dist);
    //      printf("dist[0] = %f\n", map->distance_[0]);
    //      printf("dist[100] = %f\n", map->distance_[100]);
    //      printf("query = %f %f %f\n", query.x, query.y, query.z);
    //      printf("cp = %f %f %f\n", cp.x, cp.y, cp.z);
    //    }
  }

  //  printf("vertex = %f %f %f\n", query.x, query.y, query.z);
  //
  //  printf("contactPoints = %f %f %f\n", cp.x, cp.y, cp.z);
  //
  //  printf("distance_ = %f \n", dist);
  //
  //  printf("mesh vertices = %i \n", d_nVertices);

}

__global__ void test_kernel(DMap *map, vector3 *vertices)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  //if(idx < d_nVertices)
  {
    printf("center = %f %f %f\n", map->bv_.center_.x, map->bv_.center_.y, map->bv_.center_.z);
    printf("extends = %f %f %f\n", map->bv_.extents_[0], map->bv_.extents_[1], map->bv_.extents_[2]);
    printf("mesh vertices =--------------------------------  \n");
    printf("map size = %i %i\n", map->dim_[0],map->dim_[1]);
    printf("vertex = %f %f %f\n", vertices[0].x, vertices[0].y, vertices[0].z);
  }

  //  printf("vertex = %f %f %f\n", query.x, query.y, query.z);
  //
  //  printf("contactPoints = %f %f %f\n", cp.x, cp.y, cp.z);
  //
  //  printf("distance_ = %f \n", dist);
  //
  //  printf("mesh vertices = %i \n", d_nVertices);

}

void copy_distancemap(DistanceMap<double,cpu> *map)
{

  DMap map_;

  map_.dim_[0] = map->dim_[0];
  map_.dim_[1] = map->dim_[1];

  map_.cells_[0] = map->cells_[0];
  map_.cells_[1] = map->cells_[1];
  map_.cells_[2] = map->cells_[2];

  map_.cellSize_ = map->cellSize_; 

  Vector3<float> vmin, vmax;
  vmin.x = (float)map->boundingBox_.vertices_[0].x;
  vmin.y = (float)map->boundingBox_.vertices_[0].y;
  vmin.z = (float)map->boundingBox_.vertices_[0].z;

  vmax.x = (float)map->boundingBox_.vertices_[1].x;
  vmax.y = (float)map->boundingBox_.vertices_[1].y;
  vmax.z = (float)map->boundingBox_.vertices_[1].z;

  map_.bv_.init(vmin, vmax);

  cudaMalloc((void**)&d_map, sizeof(DMap));
  cudaCheckErrors("Allocate dmap");

  cudaMemcpy(d_map, &map_, sizeof(DMap), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy distancemap class");

  Vector3<float> *vertexCoords;
  Vector3<float> *normals;
  Vector3<float> *contactPoints;      

  float *distance_;

  int size = map->dim_[0] * map->dim_[1]; 

  map->outputInfo();

  vertexCoords = new Vector3<float>[size];
  normals = new Vector3<float>[size];
  contactPoints = new Vector3<float>[size];
  distance_ = new float[size];

  for (int i = 0; i < size; i++)
  {
    vertexCoords[i].x = (float)map->vertexCoords_[i].x;
    vertexCoords[i].y = (float)map->vertexCoords_[i].y;
    vertexCoords[i].z = (float)map->vertexCoords_[i].z;

    normals[i].x = (float)map->normals_[i].x;
    normals[i].y = (float)map->normals_[i].y;
    normals[i].z = (float)map->normals_[i].z;

    contactPoints[i].x = (float)map->contactPoints_[i].x;
    contactPoints[i].y = (float)map->contactPoints_[i].y;
    contactPoints[i].z = (float)map->contactPoints_[i].z;

    distance_[i] = (float)map->distance_[i];
  }

  cudaMalloc((void**)&d_vertexCoords, size * sizeof(vector3));
  cudaCheckErrors("Allocate vertices distancemap");

  cudaMemcpy(d_vertexCoords, vertexCoords, size * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices distance");

  cudaMemcpy(&d_map->vertices_ , &d_vertexCoords, sizeof(vector3*), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices distance");

  cudaMalloc((void**)&d_normals, size * sizeof(vector3));
  cudaCheckErrors("Allocate vertices normals");

  cudaMemcpy(d_normals, normals, size * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices normals");

  cudaMemcpy(&d_map->normals_ , &d_normals, sizeof(vector3*), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices normals");

  cudaMalloc((void**)&d_contactPoints, size * sizeof(vector3));
  cudaCheckErrors("Allocate vertices contactPoints");

  cudaMemcpy(d_contactPoints, contactPoints, size * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices contactPoints");

  cudaMemcpy(&d_map->contactPoints_ , &d_contactPoints, sizeof(vector3*), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices contactPoints");

  cudaMalloc((void**)&d_distance_map, size * sizeof(float));
  cudaCheckErrors("Allocate distance");

  cudaMemcpy(d_distance_map, distance_, size * sizeof(float), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy distance");

  cudaMemcpy(&d_map->distance_ , &d_distance_map, sizeof(float*), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy distance");

  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);
  test_distmap<<<(size+255)/256, 256 >>>(d_map, d_vertexCoords);
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float elapsed_time;
  cudaEventElapsedTime(&elapsed_time, start, stop);
  cudaDeviceSynchronize();
  printf("GPU distmap coll: %3.8f [ms]\n", elapsed_time);
  test_kernel<<<1,1>>>(d_map, d_vertexCoords);

  cudaDeviceSynchronize();

  hello_kernel<<<1,1>>>();
  cudaDeviceSynchronize();
  //  std::pair<Real,Vector3<Real>> res = map->queryMap(map->vertexCoords_[0]+Vector3<Real>(0.1,0,0));
  //  std::cout << "query_cpu" << map->vertexCoords_[0] << std::endl;
  //  std::cout << "cp" << res.second << std::endl;
  //  std::cout << "dist_cpu" << res.first << std::endl;
  //  std::cout << "dist[100]" <<  map->distance_[100] << std::endl;
  //  exit(0);
  CPerfTimer timer;
  timer.Start();
  for (int i = 0; i < size; i++)
  {
    map->queryMap(map->vertexCoords_[i]);
  }

  std::cout << "Elapsed time gpu[ms]:" <<  timer.GetTime() * 1000.0 << std::endl;
  delete[] vertexCoords;
  delete[] normals;
  delete[] contactPoints;
  delete[] distance_;

}

void copy_mesh(Model3D *model){

  int nTriangles = 0;
  int nVertices = 0;

  g_model = model;

  vector3  *meshVertices;

  for (auto &mesh : model->meshes_)
  {
    nVertices += mesh.numVerts_;
    meshVertices = (vector3*)malloc(sizeof(vector3)*mesh.numVerts_);
    for (int i = 0; i < mesh.vertices_.Size(); i++)
    {
      meshVertices[i].x = (real)mesh.vertices_[i].x;
      meshVertices[i].y = (real)mesh.vertices_[i].y;
      meshVertices[i].z = (real)mesh.vertices_[i].z;
    }
  }

  printf("Number of triangles: %i\n",nVertices);
  g_triangles = nVertices;

  cudaMalloc((void**)&d_vertices, nVertices * sizeof(vector3));
  cudaCheckErrors("Allocate vertices");

  cudaMemcpy(d_vertices, meshVertices, nVertices * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("Copy vertices");
  cudaDeviceSynchronize();

  cudaMemcpyToSymbol(d_nVertices, &nVertices, sizeof(int));
  cudaCheckErrors("Copy number of vertices");

  free(meshVertices);

  cudaDeviceSynchronize();

}

void my_cuda_func(Model3D *model, UnstructuredGrid<Real, DTraits> &grid){

  int nTriangles = 0;
  int nVertices = 0;

  g_model = model;

  triangle *meshTriangles;
  vector3  *meshVertices;

  for (auto &mesh : model->meshes_)
  {
    nTriangles += mesh.numFaces_;
    meshTriangles=(triangle*)malloc(sizeof(triangle)*mesh.numFaces_);
    for (int i = 0; i < mesh.faces_.Size(); i++)
    {
      meshTriangles[i].idx0 = mesh.faces_[i][0];
      meshTriangles[i].idx1 = mesh.faces_[i][1];
      meshTriangles[i].idx2 = mesh.faces_[i][2];
    }

    nVertices += mesh.numVerts_;
    meshVertices = (vector3*)malloc(sizeof(vector3)*mesh.numVerts_);
    for (int i = 0; i < mesh.vertices_.Size(); i++)
    {
      meshVertices[i].x = (real)mesh.vertices_[i].x;
      meshVertices[i].y = (real)mesh.vertices_[i].y;
      meshVertices[i].z = (real)mesh.vertices_[i].z;
    }
  }

  model->meshes_[0].generateTriangleBoundingBoxes();

  AABB3f *boxes = new AABB3f[nTriangles];
  cudaMalloc((void**)&d_boundingBoxes, sizeof(AABB3f)* nTriangles);
  for (int i = 0; i < nTriangles; i++)
  {
    vector3 vmin, vmax;
    vmin.x = (real)model->meshes_[0].triangleAABBs_[i].vertices_[0].x;
    vmin.y = (real)model->meshes_[0].triangleAABBs_[i].vertices_[0].y;
    vmin.z = (real)model->meshes_[0].triangleAABBs_[i].vertices_[0].z;

    vmax.x = (real)model->meshes_[0].triangleAABBs_[i].vertices_[1].x;
    vmax.y = (real)model->meshes_[0].triangleAABBs_[i].vertices_[1].y;
    vmax.z = (real)model->meshes_[0].triangleAABBs_[i].vertices_[1].z;

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

