
template <typename T>
class BVHNode
{
public:

  //the node's bounding volume
  AABB3f bv_;

  int nTriangles_;

  int *indices_;

};

class DMap
{
public:
  
  //the node's bounding volume
  AABB3f bv_;

  vector3 *vertices_;
  vector3 *normals_;
  vector3 *contactPoints_;

  float* distance_;

  int *stateFBM_;

  int cells_[3];

  int dim_[2];

  float cellSize_;

  __device__ __host__
  float trilinearInterpolateDistance(const vector3 &vQuery, int indices[8]);
  
  __device__ __host__
  vector3 trilinearInterpolateCP(const vector3 &vQuery, int indices[8]);
  
  __device__ __host__
  void vertexIndices(int icellx,int icelly, int icellz, int indices[8]);
  
  //QueryDistanceMap
  __device__ __host__
  void queryMap(const vector3 &vQuery, float &first, vector3 &second);

};

__device__ __host__
void DMap::vertexIndices(int icellx,int icelly, int icellz, int indices[8])
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

__device__ __host__
float DMap::trilinearInterpolateDistance(const vector3 &vQuery, int indices[8])
{
  //trilinear interpolation of distance
  float x_d= (vQuery.x - vertices_[indices[0]].x)/(vertices_[indices[1]].x - vertices_[indices[0]].x);
  float y_d= (vQuery.y - vertices_[indices[0]].y)/(vertices_[indices[2]].y - vertices_[indices[0]].y);
  float z_d= (vQuery.z - vertices_[indices[0]].z)/(vertices_[indices[4]].z - vertices_[indices[0]].z);  
  
  float c00 = distance_[indices[0]] * (1.0 - x_d) + distance_[indices[1]] * x_d;
  
  float c10 = distance_[indices[3]] * (1.0 - x_d) + distance_[indices[2]] * x_d;
  
  float c01 = distance_[indices[4]] * (1.0 - x_d) + distance_[indices[5]] * x_d;
  
  float c11 = distance_[indices[7]] * (1.0 - x_d) + distance_[indices[6]] * x_d;      
  
  //next step
  float c0 = c00*(1.0 - y_d) + c10 * y_d;
  
  float c1 = c01*(1.0 - y_d) + c11 * y_d;  
  
  //final step
  float c = c0*(1.0 - z_d) + c1 * z_d;  
  
  return c;
}

__device__ __host__
vector3 DMap::trilinearInterpolateCP(const vector3 &vQuery, int indices[8])
{
  //trilinear interpolation of distance
  float x_d= (vQuery.x - vertices_[indices[0]].x)/(vertices_[indices[1]].x - vertices_[indices[0]].x);
  float y_d= (vQuery.y - vertices_[indices[0]].y)/(vertices_[indices[2]].y - vertices_[indices[0]].y);
  float z_d= (vQuery.z - vertices_[indices[0]].z)/(vertices_[indices[4]].z - vertices_[indices[0]].z);  
  
  vector3 c00 = contactPoints_[indices[0]] * (1.0 - x_d) + contactPoints_[indices[1]] * x_d;

  vector3 c10 = contactPoints_[indices[3]] * (1.0 - x_d) + contactPoints_[indices[2]] * x_d;

  vector3 c01 = contactPoints_[indices[4]] * (1.0 - x_d) + contactPoints_[indices[5]] * x_d;

  vector3 c11 = contactPoints_[indices[7]] * (1.0 - x_d) + contactPoints_[indices[6]] * x_d;      
  
  //next step
  vector3 c0 = c00*(1.0 - y_d) + c10 * y_d;
  
  vector3 c1 = c01*(1.0 - y_d) + c11 * y_d;  
  
  //final step
  vector3 c = c0*(1.0 - z_d) + c1 * z_d;  
  
  return c;
}

__device__ __host__
void DMap::queryMap(const vector3 &vQuery, float &first, vector3 &second)
{

  vector3 normal;
  vector3 center;
  vector3 origin;  

  //std::pair<T,Vector3<T> > res(dist,normal);
  
  if(!bv_.isPointInside(vQuery))
  {
    first = -1.0f;
    return; 
  }
  
  //calculate the cell indices
  float invCellSize = 1.0/cellSize_;
  
  //calculate the cell indices
  int x = (int)(fabs(vQuery.x-bv_.vertices_[0].x) * invCellSize);
  int y = (int)(fabs(vQuery.y-bv_.vertices_[0].y) * invCellSize);
  int z = (int)(fabs(vQuery.z-bv_.vertices_[0].z) * invCellSize);  
  
  //vertex indices
  int indices[8];
  
  //look up distance for the cell
  vertexIndices(x,y,z,indices);
   
  center=center*0.125;
  
  first  = trilinearInterpolateDistance(vQuery,indices);
  second = trilinearInterpolateCP(vQuery,indices);
  
  //return res;
}
