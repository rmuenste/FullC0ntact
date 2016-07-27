#ifndef DISTANCEMAP_CUH_QZ8HKFO4
#define DISTANCEMAP_CUH_QZ8HKFO4

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

        __device__  
        void info()
        {
          printf("boundingBox:\n");
          printf("center = %f %f %f \n", boundingBox_.center_.x,
                                         boundingBox_.center_.y, 
                                         boundingBox_.center_.z);               

          printf("extents = %f %f %f \n",boundingBox_.extents_[0],
                                         boundingBox_.extents_[1], 
                                         boundingBox_.extents_[2]);               

          printf("vertex0 = %f %f %f \n",boundingBox_.vertices_[0].x,
                                         boundingBox_.vertices_[0].y, 
                                         boundingBox_.vertices_[0].z);               
          printf("vertex1 = %f %f %f \n",boundingBox_.vertices_[1].x,
                                         boundingBox_.vertices_[1].y, 
                                         boundingBox_.vertices_[1].z);               
        };
               
        void transferData(const DistanceMap<float,cpu> &map_)
        {

          vector3 *dev_vertexCoords;
          vector3 *dev_normals;
          vector3 *dev_contactPoints;      
          float   *dev_distance;

          int size = map_.dim_[0] * map_.dim_[1]; 

          cudaMalloc((void**)&dev_vertexCoords, size * sizeof(vector3));
          cudaCheckErrors("Allocate vertices distancemap");

          cudaMemcpy(dev_vertexCoords, map_.vertexCoords_, size * sizeof(vector3), cudaMemcpyHostToDevice);
          cudaCheckErrors("copy vertices distance");

          cudaMemcpy(&vertexCoords_ , &dev_vertexCoords, sizeof(vector3*), cudaMemcpyHostToDevice);
          cudaCheckErrors("copy vertices distance");

          cudaMalloc((void**)&dev_normals, size * sizeof(vector3));
          cudaCheckErrors("Allocate vertices normals");

          cudaMemcpy(dev_normals, map_.normals_, size * sizeof(vector3), cudaMemcpyHostToDevice);
          cudaCheckErrors("copy vertices normals");

          cudaMemcpy(&normals_ , &dev_normals, sizeof(vector3*), cudaMemcpyHostToDevice);
          cudaCheckErrors("copy vertices normals");

          cudaMalloc((void**)&dev_contactPoints, size * sizeof(vector3));
          cudaCheckErrors("Allocate vertices contactPoints");

          cudaMemcpy(dev_contactPoints, map_.contactPoints_, size * sizeof(vector3), cudaMemcpyHostToDevice);
          cudaCheckErrors("copy vertices contactPoints");

          cudaMemcpy(&contactPoints_ , &dev_contactPoints, sizeof(vector3*), cudaMemcpyHostToDevice);
          cudaCheckErrors("copy vertices contactPoints");

          cudaMalloc((void**)&dev_distance, size * sizeof(float));
          cudaCheckErrors("Allocate distance");

          cudaMemcpy(dev_distance, map_.distance_, size * sizeof(float), cudaMemcpyHostToDevice);
          cudaCheckErrors("copy distance");

          cudaMemcpy(&distance_, &dev_distance, sizeof(float*), cudaMemcpyHostToDevice);
          cudaCheckErrors("copy distance");
        }

        __device__ __host__ 
          //ClosestPoint to vertex -> easily compute normal
          T trilinearInterpolateDistance(const Vector3<T> &vQuery, int indices[8]);

        __device__ __host__
          Vector3<T> trilinearInterpolateCP(const Vector3<T> &vQuery, int indices[8]);

        __device__ __host__
          void vertexIndices(int icellx,int icelly, int icellz, int indices[8]);

        __device__ __host__
          void queryMap(const vector3 &vQuery, float &first, vector3 &second);
          
        __device__ __host__          
          int queryFBM(const vector3 &vQuery);          

       __device__ __host__ void queryMap(const vector3 &vQuery, float &first,
                                    vector3 &second,
                                    vector3 &n);
    };

  template<typename T>
    __device__ __host__
    void DistanceMap<T,gpu>::queryMap(const vector3 &vQuery, float &first, vector3 &second)
    {

      vector3 normal;
      vector3 center;
      vector3 origin;  

      //std::pair<T,Vector3<T> > res(dist,normal);

      if(!boundingBox_.isPointInside(vQuery))
      {
        first = 5.0f;
        return; 
      }

      //calculate the cell indices
      float invCellSize = 1.0/cellSize_;

      //calculate the cell indices
      int x = (int)(fabs(vQuery.x-boundingBox_.vertices_[0].x) * invCellSize);
      int y = (int)(fabs(vQuery.y-boundingBox_.vertices_[0].y) * invCellSize);
      int z = (int)(fabs(vQuery.z-boundingBox_.vertices_[0].z) * invCellSize);  

      //vertex indices
      int indices[8];

      //look up distance for the cell
      vertexIndices(x,y,z,indices);

      first  = trilinearInterpolateDistance(vQuery,indices);
      second = trilinearInterpolateCP(vQuery,indices);

    }

  template<typename T>
    __device__ __host__
    void DistanceMap<T,gpu>::queryMap(const vector3 &vQuery, float &first,
                                              vector3 &second,
                                              vector3 &n)
    {

      vector3 normal;
      vector3 center;
      vector3 origin;  

      //std::pair<T,Vector3<T> > res(dist,normal);

      if(!boundingBox_.isPointInside(vQuery))
      {
        first = 5.0f;
        return; 
      }

      //calculate the cell indices
      float invCellSize = 1.0/cellSize_;

      //calculate the cell indices
      int x = (int)(fabs(vQuery.x-boundingBox_.vertices_[0].x) * invCellSize);
      int y = (int)(fabs(vQuery.y-boundingBox_.vertices_[0].y) * invCellSize);
      int z = (int)(fabs(vQuery.z-boundingBox_.vertices_[0].z) * invCellSize);  

      //vertex indices
      int indices[8];

      //look up distance for the cell
      vertexIndices(x,y,z,indices);

      first  = trilinearInterpolateDistance(vQuery,indices);
      second = trilinearInterpolateCP(vQuery,indices);
      n      = normals_[indices[0]];

    }
    
  template<typename T>
    __device__ __host__
    int DistanceMap<T,gpu>::queryFBM(const vector3 &vQuery)
    {
    
      if(!boundingBox_.isPointInside(vQuery))
      {
        return 0; 
      }

      //calculate the cell indices
      float invCellSize = 1.0/cellSize_;

      //calculate the cell indices
      int x = (int)(fabs(vQuery.x-boundingBox_.vertices_[0].x) * invCellSize);
      int y = (int)(fabs(vQuery.y-boundingBox_.vertices_[0].y) * invCellSize);
      int z = (int)(fabs(vQuery.z-boundingBox_.vertices_[0].z) * invCellSize);  

      //vertex indices
      int indices[8];

      //look up distance for the cell
      vertexIndices(x,y,z,indices);

      T first  = trilinearInterpolateDistance(vQuery,indices);      
      //printf(" vertexCoords = %f %f %f = %f\n", vQuery.x, vQuery.y, first);
            
      if(first <= T(0.0))
        return 1;
      else
        return 0;

    }    

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


#endif /* end of include guard: DISTANCEMAP_CUH_QZ8HKFO4 */
