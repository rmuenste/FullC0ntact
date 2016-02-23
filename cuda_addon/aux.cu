inline float frand()
{
  return rand() / (float)RAND_MAX;
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

__global__ void test_kernel(DistanceMap<float,gpu> *map, vector3 *vertices)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  //if(idx < d_nVertices)
  {
    printf("center = %f %f %f\n", map->boundingBox_.center_.x, map->boundingBox_.center_.y, map->boundingBox_.center_.z);
    printf("extends = %f %f %f\n", map->boundingBox_.extents_[0], map->boundingBox_.extents_[1], map->boundingBox_.extents_[2]);
    printf("mesh vertices =--------------------------------  \n");
    printf("map size = %i %i\n", map->dim_[0],map->dim_[1]);
    printf("vertex = %f %f %f\n", vertices[0].x, vertices[0].y, vertices[0].z);
  }

}

__global__ void gpu_map_test(DistanceMap<float, gpu> *map)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  //if(idx < d_nVertices)
  {
    printf("center = %f %f %f\n", map->boundingBox_.center_.x, map->boundingBox_.center_.y, map->boundingBox_.center_.z);
    printf("extends = %f %f %f\n", map->boundingBox_.extents_[0], map->boundingBox_.extents_[1], map->boundingBox_.extents_[2]);
    printf("map size = %i %i\n", map->dim_[0], map->dim_[1]);
  }

}

__global__ void test_kernel(DistanceMap<float,gpu> *map)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  //if(idx < d_nVertices)
  {
    printf("center = %f %f %f\n", map->boundingBox_.center_.x, map->boundingBox_.center_.y, map->boundingBox_.center_.z);
    printf("extends = %f %f %f\n", map->boundingBox_.extents_[0], map->boundingBox_.extents_[1], map->boundingBox_.extents_[2]);
    printf("mesh vertices =--------------------------------  \n");
    printf("map size = %i %i\n", map->dim_[0],map->dim_[1]);
    printf("vertexCoords = %f %f %f\n", map->vertexCoords_[0].x, map->vertexCoords_[0].y, map->vertexCoords_[0].z);
  }

}

__global__ void test_dist(DistanceMap<float,gpu> *map, vector3 *vectors, float *res)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(idx < NN)
  {
    vector3 query = vectors[idx];
    vector3 cp(0,0,0);
    float dist=0.0;
    map->queryMap(query,dist,cp);
    res[idx] = dist;
  }

}


//void dmap_test(RigidBody *body)
//{
//
//  int size = body->map_->dim_[0] * body->map_->dim_[1];
//
//  vector3 *testVectors = new vector3[NN];
//  vector3 *d_testVectors;
//
//  Real *distance_res = new Real[NN];
//  float *d_distance_res;
//  float *distance_gpu = new float[NN];
//
//  for(int i=0; i < NN; i++)
//  {
//    vector3 vr(0,0,0);
//    vr.x = -body->map_->boundingBox_.extents_[0] + frand() * (2.0 * body->map_->boundingBox_.extents_[0]); 
//    vr.y = -body->map_->boundingBox_.extents_[1] + frand() * (2.0 * body->map_->boundingBox_.extents_[1]); 
//    vr.z = -body->map_->boundingBox_.extents_[2] + frand() * (2.0 * body->map_->boundingBox_.extents_[2]); 
//    testVectors[i] = vr;
//  }
//
//  cudaMalloc((void**)&(d_testVectors), NN*sizeof(vector3));
//  cudaMemcpy(d_testVectors, testVectors, NN*sizeof(vector3), cudaMemcpyHostToDevice);
//
//  cudaMalloc((void**)&(d_distance_res), NN*sizeof(float));
//
//  cudaEvent_t start, stop;
//  cudaEventCreate(&start);
//  cudaEventCreate(&stop);
//  cudaEventRecord(start, 0);
//
//  test_dist<<< (NN+255)/256, 256 >>>(body->map_gpu_,d_testVectors, d_distance_res);
//  cudaMemcpy(distance_gpu, d_distance_res, NN*sizeof(float), cudaMemcpyDeviceToHost);
//  cudaEventRecord(stop, 0);
//  cudaEventSynchronize(stop);
//  float elapsed_time;
//  cudaEventElapsedTime(&elapsed_time, start, stop);
//  cudaDeviceSynchronize();
//  printf("Elapsed time gpu distmap: %3.8f [ms].\n", elapsed_time);
//
//  CPerfTimer timer;
//  timer.Start();
//  for (int i = 0; i < NN; i++)
//  {
//    Vector3<Real> v(0,0,0);
//    v.x=testVectors[i].x;
//    v.y=testVectors[i].y;
//    v.z=testVectors[i].z;
//    std::pair<Real, Vector3<Real>> res = body->map_->queryMap(v);
//    distance_res[i] = res.first;
//  }
//
//  std::cout << "Elapsed time cpu distmap: " <<  timer.GetTime() << " [ms]." << std::endl;
//
//  cudaDeviceSynchronize();
//
//  //for (int i = 0; i < NN; i++)
//  //{
//  //  printf("gpu = %3.8f cpu = %3.8f \n", distance_gpu[i], distance_res[i]);
//  //}
//
//  delete[] testVectors;
//  delete[] distance_res;
//  delete[] distance_gpu;
//
//  cudaFree(d_testVectors);
//  cudaFree(d_distance_res);
//
//}
