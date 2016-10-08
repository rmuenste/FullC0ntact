#ifndef UNIT_TESTS_CUH_QYLHZMAK
#define UNIT_TESTS_CUH_QYLHZMAK

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

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  vector3 orig(0.0f, 0.0f, 0.0f);
  vector3 dir(0.9, 0.8, 0.02);

  vector3 v0(0.474477, 0.268391, 0.162952);
  vector3 v1(0.393, 0.263599, -0.004793);
  vector3 v2(0.297147, 0.335489, 0.014378);

  if (intersection_tri(orig, dir, v0, v1, v2, idx))
  {
    printf("GPU: intersection \n");
  }

}

__global__ void test_single(vector3 query, triangle *triangles, vector3* vertices, int *i)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  vector3 dir(0.10, 0.0, 1.0);
  i[0]=0;
  if(idx < d_nTriangles)
  {
    vector3 &v0 = vertices[triangles[idx].idx0];
    vector3 &v1 = vertices[triangles[idx].idx1];
    vector3 &v2 = vertices[triangles[idx].idx2];
    if (intersection_tri(query, dir, v0, v1, v2, idx))
    {
      i[0]=1;
    }
  }

}

void single_triangle_test(UnstructuredGrid<Real, DTraits> &grid)
{
  int intersection[threadsPerBlock];
  int *d_intersection;
  
  cudaMalloc((void**)&d_intersection,sizeof(int)*threadsPerBlock);
  for (int j=0; j < grid.nvt_; j++)
  {
    int id = j; 
    VECTOR3 vQuery = grid.vertexCoords_[j];
    int intersections = 0;
    cudaMemset(&d_intersection,0, sizeof(int)*threadsPerBlock); 
    vector3 query((real)vQuery.x, (real)vQuery.y, (real)vQuery.z);
    intersection[0]=0;
    for(int i=0; i < g_triangles; i++)
    {
      test_single<<<1,1>>> (query, d_triangles, d_vertices, d_intersection);
      cudaDeviceSynchronize();
      cudaMemcpy(&intersection, d_intersection, sizeof(int)*threadsPerBlock, cudaMemcpyDeviceToHost);
      if(intersection[0])
        intersections++;

//      printf("with triangle : %i intersections: %i \n",i,intersections);
    }
    if (intersections%2!=0)
    {
      grid.m_myTraits[id].iTag = 1;
    }
    else
    {
      grid.m_myTraits[id].iTag = 0;
    }

  }
}

__device__ bool intersection2(const vector3 &orig, const vector3 &dir, const vector3 &v0, const vector3 &v1, const vector3 &v2)
{


  float t;

  // compute plane's normal
  vector3 v0v1 = v1 - v0;
  vector3 v0v2 = v2 - v0;
  // no need to normalize
  vector3 N = vector3::Cross(v0v1, v0v2); // N 

  // Step 1: finding P

  // check if ray and plane are parallel ?
  float NdotRayDirection = N * dir;
  if (fabs(NdotRayDirection) < 0.0000005) //1e-6f) // almost 0 
    return false; // they are parallel so they don't intersect ! 

  // compute d parameter using equation 2
  float d = N*v0;

  // compute t (equation 3)
  t = (N*orig + d) / NdotRayDirection;
  // check if the triangle is in behind the ray
  if (t < 0) return false; // the triangle is behind 

  // compute the intersection point using equation 1
  vector3 P = orig + dir * t;

  // Step 2: inside-outside test
  vector3 C; // vector perpendicular to triangle's plane 

  // edge 0
  vector3 edge0 = v1 - v0;
  vector3 vp0 = P - v0;
  C = vector3::Cross(edge0, vp0);
  if (N * C < 0) return false; // P is on the right side 

  // edge 1
  vector3 edge1 = v2 - v1;
  vector3 vp1 = P - v1;
  C = vector3::Cross(edge1, vp1);
  if (N * C < 0)  return false; // P is on the right side 

  // edge 2
  vector3 edge2 = v0 - v2;
  vector3 vp2 = P - v2;
  C = vector3::Cross(edge2, vp2);
  if (N * C < 0) return false; // P is on the right side; 

  return true; // this ray hits the triangle 
}


__global__ void test_all_triangles(vector3 query, triangle *triangles, vector3* vertices, int *nintersections)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  //Vector3<float> dir_(0.1, 0.1, 1.0);
  //dir_.Normalize();
  //vector3 dir(dir_.x,dir_.y,dir_.z);
  vector3 dir(1.0, 0, 0.0);

  nintersections[idx] = 0;
  if (idx < d_nTriangles)
  {
    vector3 &v0 = vertices[triangles[idx].idx0];
    vector3 &v1 = vertices[triangles[idx].idx1];
    vector3 &v2 = vertices[triangles[idx].idx2];
    if (intersection_tri(query, dir, v0, v1, v2, idx))
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
  VECTOR3 vQuery(0, 0, 0);
#endif
  cudaMemset(&d_intersection, 0, sizeof(int)*threadsPerBlock);
  int intersections = 0;
  vector3 query((real)vQuery.x, (real)vQuery.y, (real)vQuery.z);
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

void triangle_test(UnstructuredGrid<Real, DTraits> &grid)
{

//  int *d_intersection;
//  int intersection[threadsPerBlock];
//
//  cudaMalloc((void**)&d_intersection, sizeof(int)*threadsPerBlock);
//  CPerfTimer timer;
//  timer.Start();
//  cudaEvent_t start, stop;
//  cudaEventCreate(&start);
//  cudaEventCreate(&stop);
//  cudaEventRecord(start, 0);
//  for (int j = 0; j < grid.nvt_; j++)
//  {
//    int id = j;
//    VECTOR3 vQuery = grid.vertexCoords_[j];
//    cudaMemset(&d_intersection, 0, sizeof(int)*threadsPerBlock);
//    int intersections = 0;
//    vector3 query((real)vQuery.x, (real)vQuery.y, (real)vQuery.z);
//    if (!g_model->GetBox().isPointInside(vQuery))
//    {
//      continue;
//    }
//
//    test_all_triangles << <1, threadsPerBlock >> > (query, d_triangles, d_vertices, d_intersection);
//    cudaMemcpy(&intersection, d_intersection, sizeof(int)*threadsPerBlock, cudaMemcpyDeviceToHost);
//
//    for (int i = 0; i < threadsPerBlock; i++)
//    {
//      intersections += intersection[i];
//    }
//    if (intersections % 2 != 0)
//    {
//      grid.m_myTraits[id].iTag = 1;
//    }
//    else
//    {
//      grid.m_myTraits[id].iTag = 0;
//    }
//
//  }
//  cudaEventRecord(stop, 0);
//  cudaEventSynchronize(stop);
//  float elapsed_time;
//  cudaEventElapsedTime(&elapsed_time, start, stop);
//  cudaDeviceSynchronize();
//  double dt_gpu = timer.GetTime();
//  //std::cout << "nIntersections: " << nIntersections << std::endl;
//  //std::cout << "GPU time: " << dt_gpu << std::endl;
//  printf("GPU time: %3.8f ms\n", dt_gpu);
//  printf("GPU time event: %3.8f ms\n", elapsed_time);

}

inline void test_inside(int cpu, int gpu)
{
  if (gpu == cpu)
  {
    std::cout << "> Test 4: OK! inside on gpu = inside on cpu = " << gpu << std::endl;
  }
  else
  {
    std::cout << "> Test 4: failed! inside on gpu = " << gpu << " != inside on cpu = " << cpu << std::endl;
  }
}


__global__ void test_dist_comp(DistanceMap<float, gpu> *map, float cpu_dist)
{

  vector3 query(0.001,0,0);
  vector3 cp(0, 0, 0);
  float dist = 0.0;
  map->queryMap(query, dist, cp);
  if (fabs((dist - cpu_dist) / (cpu_dist)) < 1e-3)
  {
    printf("> Test 3: distmap comparision OK!\n");
  }
  else
  {
    printf("> Test 3: distmap comparision failed!\n");
  }

}

__global__ void test_dist_size(DistanceMap<float, gpu> *map)
{

  map->info();

}



#endif /* end of include guard: UNIT_TESTS_CUH_QYLHZMAK */
