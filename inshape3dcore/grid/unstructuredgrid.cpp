#include "unstructuredgrid.h"

namespace i3d {

template<typename T>
bool SameSign(T a, T b)
{
    if (std::abs(a) == 0.0 || std::abs(b) == 0.0)
        return true;

    if (std::abs(a) < 1e-3 || std::abs(b)  < 1e-3)
        return true;

    return (a >= 0.0) == (b >= 0.0);
}

struct coordx
{
  bool operator()(const sPerm &elem1,const sPerm &elem2 ) 
  {
  return elem1.vVec.x < elem2.vVec.x;
  }
};

struct coordy
{
  bool operator()(const sPerm &elem1,const sPerm &elem2 ) 
  {
  return elem1.vVec.y < elem2.vVec.y;
  }
};  

struct coordz
{
  bool operator()(const sPerm &elem1,const sPerm &elem2 ) 
  {
  return elem1.vVec.z < elem2.vVec.z;
  }
};  
  
template<class T,class Traits>
UnstructuredGrid<T,Traits>::UnstructuredGrid(void)
{
  vertexCoords_         = NULL;
  hexas_                = NULL;
  verticesAtBoundary_   = NULL;
  facesAtBoundary_      = NULL;
  elementsAtBoundary_   = NULL;
  verticesAtFace_       = NULL;
  verticesAtEdge_       = NULL;
  neighborsAtElement_   = NULL;
  m_VertexVertex        = NULL;
  m_myTraits            = NULL;
  elementsAtVertexIdx_  = NULL;
  elementsAtVertex_     = NULL;
  vertexOrder_          = NULL;
  refinementLevel_ = 0;
};

template<class T,class Traits>
UnstructuredGrid<T,Traits>::~UnstructuredGrid(void)
{
  
  if(vertexCoords_ != NULL)
  {
    delete[] vertexCoords_;    
    vertexCoords_ = NULL;
  }
  
  if(vertexOrder_ != NULL)
  {
    delete[] vertexOrder_;    
    vertexOrder_ = NULL;
  }  
  
  if(hexas_ != NULL)
  {
    delete[] hexas_;
    hexas_ = NULL;
  }
  
  if(verticesAtBoundary_ != NULL)
  {
    delete[] verticesAtBoundary_;    
    verticesAtBoundary_ = NULL;
  }  
  
  if(facesAtBoundary_ != NULL)
  {
    delete[] facesAtBoundary_;
    facesAtBoundary_ = NULL;
  }
  
  if(elementsAtBoundary_ != NULL)
  {
    delete[] elementsAtBoundary_;
    elementsAtBoundary_ = NULL;
  }  
  
  if(verticesAtFace_ != NULL)
  {
    delete[] verticesAtFace_;
    verticesAtFace_ = NULL;
  }  

  if(verticesAtEdge_ != NULL)
  {
    delete[] verticesAtEdge_;
    verticesAtEdge_ = NULL;
  }  

  if(neighborsAtElement_ != NULL)
  {
    delete[] neighborsAtElement_;
    neighborsAtElement_ = NULL;
  }  

  if(m_VertexVertex != NULL)
  {
    delete[] m_VertexVertex;
    m_VertexVertex = NULL;
  }  

  if(m_myTraits != NULL)
  {
    delete[] m_myTraits;
    m_myTraits = NULL;
  }  

  if(elementsAtVertexIdx_ != NULL)
  {
    delete[] elementsAtVertexIdx_;
    elementsAtVertexIdx_ = NULL;
  }
  
  if(elementsAtVertex_ != NULL)
  {
    delete[] elementsAtVertex_;
    elementsAtVertex_ = NULL;
  }    
  
  for (int i = 0; i < verticesAtHexaLev_.size(); i++)
  {
    delete[] verticesAtEdgeLev_[i];
    delete[] verticesAtFaceLev_[i];
    delete[] verticesAtHexaLev_[i];
    delete[] verticesAtBoundaryLev_[i];
  }

};

template<class T,class Traits>
void UnstructuredGrid<T,Traits>::initUnitCube()
{
  // the total number of vertices
  nvt_ = 8;

  // the total number of elements
  nel_ = 1;

  this->vertexCoords_ = new Vector3<T>[nvt_+1];
  verticesAtBoundary_ = new int[nvt_];  
  hexas_              = new Hexa[1];

  m_myTraits            = new Traits[nvt_];
  for(int i=0;i<nvt_;i++)
    memset(&m_myTraits[i],0,sizeof(Traits));

  vertexCoords_[0] = Vector3<T>(-1.0,-1.0,-1.0);
  vertexCoords_[1] = Vector3<T>(1.0,-1.0,-1.0);
  vertexCoords_[2] = Vector3<T>(1.0,1.0,-1.0);
  vertexCoords_[3] = Vector3<T>(-1.0,1.0,-1.0);
  vertexCoords_[4] = Vector3<T>(-1.0,-1.0,1.0);
  vertexCoords_[5] = Vector3<T>(1.0,-1.0,1.0);
  vertexCoords_[6] = Vector3<T>(1.0,1.0,1.0);
  vertexCoords_[7] = Vector3<T>(-1.0,1.0,1.0);
  verticesAtBoundary_[0]   = 0;
  verticesAtBoundary_[1]   = 1;
  verticesAtBoundary_[2]   = 2;
  verticesAtBoundary_[3]   = 3;
  verticesAtBoundary_[4]   = 4;
  verticesAtBoundary_[5]   = 5;
  verticesAtBoundary_[6]   = 6;
  verticesAtBoundary_[7]   = 7;

  hexas_[0].hexaVertexIndices_[0]      = 0;
  hexas_[0].hexaVertexIndices_[1]      = 1;
  hexas_[0].hexaVertexIndices_[2]      = 2;
  hexas_[0].hexaVertexIndices_[3]      = 3;
  hexas_[0].hexaVertexIndices_[4]      = 4;
  hexas_[0].hexaVertexIndices_[5]      = 5;
  hexas_[0].hexaVertexIndices_[6]      = 6;
  hexas_[0].hexaVertexIndices_[7]      = 7;

};

template<class T,class Traits>
void UnstructuredGrid<T,Traits>::calcVol()
{

  elemVol_.clear();
  ElementIter el_it = elem_begin();
  for (; el_it != elem_end(); el_it++)
  {
    int idx = el_it.idx();
    T v = elemVol(idx); 
    elemVol_.push_back(v);
  }

  vol_ = T(0.0);
  for(auto v : elemVol_)
  {
    vol_ += v; 
  }

}

template<class T,class Traits>
bool UnstructuredGrid<T,Traits>::sameSide(const Vector3<T> &query,
                                          const Vector3<T> &a, 
                                          const Vector3<T> &b,
                                          const Vector3<T> &c,
                                          const Vector3<T> &d,
                                          bool debug)
{

  Vector3<T> normal(Vector3<T>::Cross((b-a),(c-a)));
  T dot4 = normal * (d-a);
  T dotq = normal * (query-a);

  if(debug)
  {
    std::cout << "> dot4: " << dot4 << std::endl; 
    std::cout << "> dot1: " << dotq << std::endl; 
  }

  //return std::signbit(dot4) && std::signbit(dotq);
  return SameSign<T>(dot4,dotq);

}

template<class T,class Traits>
bool UnstructuredGrid<T,Traits>::pointInsideTetra(const Vector3<T> &query,
                                                  const Vector3<T> &a, 
                                                  const Vector3<T> &b,
                                                  const Vector3<T> &c,
                                                  const Vector3<T> &d,
                                                  bool debug)
{

  return sameSide(query, a, b, c, d,debug) &&
         sameSide(query, b, c, d, a,debug) &&
         sameSide(query, c, d, a, b,debug) &&
         sameSide(query, d, a, b, c,debug);

}

template<class T,class Traits>
bool UnstructuredGrid<T,Traits>::pointInsideHexa(int hIdx, const Vector3<T> &query)
{

  //
  bool inside = false;
  int tetras[5][4] = {{0,1,3,4},{1,2,3,6},{3,4,6,7},{1,4,5,6},{1,3,4,6}};
  for(int i(0); i < 5; ++i)
  {
    Vector3<T> a = vertexCoords_[hexas_[hIdx].hexaVertexIndices_[tetras[i][0]]];
    Vector3<T> b = vertexCoords_[hexas_[hIdx].hexaVertexIndices_[tetras[i][1]]];
    Vector3<T> c = vertexCoords_[hexas_[hIdx].hexaVertexIndices_[tetras[i][2]]];
    Vector3<T> d = vertexCoords_[hexas_[hIdx].hexaVertexIndices_[tetras[i][3]]];

    Tetrahedron<T> tetra(a,b,c,d);

//    if(pointInsideTetra(query,a,b,c,d))
//      return true;

    if(tetra.pointInside(query))
      return true;

  }

  return inside;

}

template<class T,class Traits>
bool UnstructuredGrid<T,Traits>::pointInsideHexaDebug(int hIdx, const Vector3<T> &query)
{

  //
  bool inside = false;
  int tetras[5][4] = {{0,1,3,4},{1,2,3,6},{3,4,6,7},{1,4,5,6},{1,3,4,6}};

  Vector3<T> minV;
  Vector3<T> maxV;
  T MaxX = -std::numeric_limits<T>::max();
  T MinX = std::numeric_limits<T>::max();
  T MaxY = -std::numeric_limits<T>::max();
  T MinY = std::numeric_limits<T>::max();
  T MaxZ = -std::numeric_limits<T>::max();
  T MinZ = std::numeric_limits<T>::max();

  Hexa *h = &hexas_[hIdx];

  for(int j(0); j < 8; ++j)
  {

    Vector3<T> vv = vertexCoords_[h->hexaVertexIndices_[j]];

    if(vv.x < MinX)
    {	//assign min index
      MinX = vv.x;
    }

    if(vv.x > MaxX)
    {	//assign max index
      MaxX = vv.x;
    }

    if(vv.y < MinY)
    {	//assign min index
      MinY = vv.y;
    }

    if(vv.y > MaxY)
    {	//assign max index
      MaxY = vv.y;
    }

    if(vv.z < MinZ)
    {	//assign min index
      MinZ = vv.z;
    }

    if(vv.z > MaxZ)
    {	//assign max index
      MaxZ = vv.z;
    }

  }

  minV.x = MinX;
  minV.y = MinY;
  minV.z = MinZ;

  maxV.x = MaxX;
  maxV.y = MaxY;
  maxV.z = MaxZ;
  AABB3<T> box(minV, maxV);
  if(box.isPointInside(query))
  {
    std::cout<<"> inside hexa box"<<std::endl;
  }

  for(int i(0); i < 5; ++i)
  {
    Vector3<T> a = vertexCoords_[hexas_[hIdx].hexaVertexIndices_[tetras[i][0]]];
    Vector3<T> b = vertexCoords_[hexas_[hIdx].hexaVertexIndices_[tetras[i][1]]];
    Vector3<T> c = vertexCoords_[hexas_[hIdx].hexaVertexIndices_[tetras[i][2]]];
    Vector3<T> d = vertexCoords_[hexas_[hIdx].hexaVertexIndices_[tetras[i][3]]];

//    if(tetra.pointInside(query))
//      return true;

    if(pointInsideTetra(query,a,b,c,d,true))
    {
      std::cout<<"> inside tetra: "<< i  <<std::endl;

      Tetrahedron<T> tetra(a,b,c,d);

      tetra.pointInsideDebug(query);
    }
    else
    {
      std::cout<<"> not inside tetra: "<< i  <<std::endl;
    }

  }

  return inside;

}

template<class T,class Traits>
bool UnstructuredGrid<T,Traits>::pointInside(const Vector3<T> &query)
{

  bool inside = false;
  ElementIter el_it = elem_begin();
  for (; el_it != elem_end(); el_it++)
  {
    int hIdx = el_it.idx();
    Vector3<T> minV;
    Vector3<T> maxV;
    T MaxX = -std::numeric_limits<T>::max();
    T MinX = std::numeric_limits<T>::max();
    T MaxY = -std::numeric_limits<T>::max();
    T MinY = std::numeric_limits<T>::max();
    T MaxZ = -std::numeric_limits<T>::max();
    T MinZ = std::numeric_limits<T>::max();

    Hexa *h = el_it.Get();

    for(int j(0); j < 8; ++j)
    {

      Vector3<T> vv = vertexCoords_[h->hexaVertexIndices_[j]];

      if(vv.x < MinX)
      {	//assign min index
        MinX = vv.x;
      }

      if(vv.x > MaxX)
      {	//assign max index
        MaxX = vv.x;
      }

      if(vv.y < MinY)
      {	//assign min index
        MinY = vv.y;
      }

      if(vv.y > MaxY)
      {	//assign max index
        MaxY = vv.y;
      }

      if(vv.z < MinZ)
      {	//assign min index
        MinZ = vv.z;
      }

      if(vv.z > MaxZ)
      {	//assign max index
        MaxZ = vv.z;
      }

    }

    minV.x = MinX;
    minV.y = MinY;
    minV.z = MinZ;

    maxV.x = MaxX;
    maxV.y = MaxY;
    maxV.z = MaxZ;
    AABB3<T> box(minV, maxV);
    if(box.isPointInside(query))
    {
      if(pointInsideHexa(hIdx, query))
        return true;
    }

  }

  return inside;

}

template<class T,class Traits>
T UnstructuredGrid<T,Traits>::elemVol(int idx)
{

  // V = |(a-d)*((b-d)x(c-d))| * (1.0/6.0)
  int tetras[5][4] = {{0,1,3,4},{1,2,3,6},{3,4,6,7},{1,4,5,6},{1,3,4,6}};
  T vol = T(0.0);
  for(int i(0); i < 5; ++i)
  {
    Vector3<T> a = vertexCoords_[hexas_[idx].hexaVertexIndices_[tetras[i][0]]];
    Vector3<T> b = vertexCoords_[hexas_[idx].hexaVertexIndices_[tetras[i][1]]];
    Vector3<T> c = vertexCoords_[hexas_[idx].hexaVertexIndices_[tetras[i][2]]];
    Vector3<T> d = vertexCoords_[hexas_[idx].hexaVertexIndices_[tetras[i][3]]];
    vol += std::abs((a-d)*(Vector3<T>::Cross((b-d),(c-d)))) * (1.0/6.0);
  }
  return vol;

}

template<class T,class Traits>
void UnstructuredGrid<T,Traits>::initMeshFromFile(const char *strFileName)
{
  using namespace std;
  ifstream in(strFileName);

  cout<<"Loading grid file: "<<strFileName<<" ..."<<endl;

  if(!in.is_open())
  {
    cout<<"Error opening file: "<<strFileName<<endl;
    exit(0);
  }

  char strLine[256];
  string first;

  in.getline(strLine,256);
  in.getline(strLine,256);
  in >> nel_;
  in >> nvt_;
  in.getline(strLine,256);
  in.getline(strLine,256);

  this->vertexCoords_ = new Vector3<T>[nvt_+1];
  hexas_              = new Hexa[nel_];
  m_myTraits            = new Traits[nvt_];
  memset(m_myTraits,0,nvt_*sizeof(Traits));
  
  for(int i=0;i<nvt_;i++)
  {
    Vector3<T> vec;
    in >> vec.x;
    in >> vec.y;
    in >> vec.z;
    vertexCoords_[i] = vec;
    in.getline(strLine,256);
  }

  maxVertex_=Vector3<T>(vertexCoords_[0].x,vertexCoords_[0].y,vertexCoords_[0].z);
  minVertex_=Vector3<T>(vertexCoords_[0].x,vertexCoords_[0].y,vertexCoords_[0].z);

  for(int i=1;i<nvt_;i++)
  {
    if(minVertex_.x > vertexCoords_[i].x)
      minVertex_.x = vertexCoords_[i].x;

    if(minVertex_.y > vertexCoords_[i].y)
      minVertex_.y = vertexCoords_[i].y;

    if(minVertex_.z > vertexCoords_[i].z)
      minVertex_.z = vertexCoords_[i].z;

    if(maxVertex_.x < vertexCoords_[i].x)
      maxVertex_.x = vertexCoords_[i].x;

    if(maxVertex_.y < vertexCoords_[i].y)
      maxVertex_.y = vertexCoords_[i].y;

    if(maxVertex_.z < vertexCoords_[i].z)
      maxVertex_.z = vertexCoords_[i].z;
  }

  in.getline(strLine,256);
  for(int i=0;i<nel_;i++)
  {
    for(int j=0;j<8;j++)
    {
      in >> hexas_[i].hexaVertexIndices_[j];
      hexas_[i].hexaVertexIndices_[j]--;
    }
    in.getline(strLine,256);
  }
  in.getline(strLine,256);

  std::vector<int> nodalProperties;
  for(int i=0;i<nvt_;i++)
  {
    int iprop;
    in >> iprop;
    nodalProperties.push_back(iprop);
    in.getline(strLine,256);
  }

  verticesAtBoundary_         = new int[nodalProperties.size()];  
  for(int i=0;i<nodalProperties.size();i++)
  {
    verticesAtBoundary_[i]=nodalProperties[i];
  }

  in.close();
  cout<<"Grid: "<<strFileName<<" loaded successfully."<<endl;
}

template<class T,class Traits>
void UnstructuredGrid<T,Traits>::initCube(T xmin, T ymin, T zmin, T xmax, T ymax, T zmax)
{
  // the total number of vertices
  nvt_ = 8;

  // the total number of elements
  nel_ = 1;

  this->vertexCoords_ = new Vector3<T>[nvt_+1];
  verticesAtBoundary_         = new int[nvt_];  
  hexas_              = new Hexa[1];

  m_myTraits            = new Traits[nvt_];
  
  maxVertex_=Vector3<T>(xmax,ymax,zmax);
  minVertex_=Vector3<T>(xmin,ymin,zmin);

  vertexCoords_[0] = Vector3<T>(xmin,ymin,zmin);
  vertexCoords_[1] = Vector3<T>(xmax,ymin,zmin);
  vertexCoords_[2] = Vector3<T>(xmax,ymax,zmin);
  vertexCoords_[3] = Vector3<T>(xmin,ymax,zmin);
  vertexCoords_[4] = Vector3<T>(xmin,ymin,zmax);
  vertexCoords_[5] = Vector3<T>(xmax,ymin,zmax);
  vertexCoords_[6] = Vector3<T>(xmax,ymax,zmax);
  vertexCoords_[7] = Vector3<T>(xmin,ymax,zmax);
  verticesAtBoundary_[0]   = 1;
  verticesAtBoundary_[1]   = 1;
  verticesAtBoundary_[2]   = 1;
  verticesAtBoundary_[3]   = 1;
  verticesAtBoundary_[4]   = 1;
  verticesAtBoundary_[5]   = 1;
  verticesAtBoundary_[6]   = 1;
  verticesAtBoundary_[7]   = 1;

  hexas_[0].hexaVertexIndices_[0]      = 0;
  hexas_[0].hexaVertexIndices_[1]      = 1;
  hexas_[0].hexaVertexIndices_[2]      = 2;
  hexas_[0].hexaVertexIndices_[3]      = 3;
  hexas_[0].hexaVertexIndices_[4]      = 4;
  hexas_[0].hexaVertexIndices_[5]      = 5;
  hexas_[0].hexaVertexIndices_[6]      = 6;
  hexas_[0].hexaVertexIndices_[7]      = 7;

};

template<class T, class Traits>
void UnstructuredGrid<T,Traits>::initCubeFromAABB(const AABB3<T> &aabb)
{
  // the total number of vertices
  nvt_ = 8;

  // the total number of elements
  nel_ = 1;

  this->vertexCoords_ = new Vector3<T>[nvt_+1];
  verticesAtBoundary_         = new int[nvt_];  
  hexas_              = new Hexa[1];

  m_myTraits            = new Traits[nvt_];
  
  maxVertex_=aabb.vertices_[1];
  minVertex_=aabb.vertices_[0];

  vertexCoords_[0] = minVertex_;                                             
  vertexCoords_[1] = Vector3<T>(maxVertex_.x,minVertex_.y,minVertex_.z);            
  vertexCoords_[2] = Vector3<T>(maxVertex_.x,maxVertex_.y,minVertex_.z);            
  vertexCoords_[3] = Vector3<T>(minVertex_.x,maxVertex_.y,minVertex_.z);            
  vertexCoords_[4] = Vector3<T>(minVertex_.x,minVertex_.y,maxVertex_.z);            
  vertexCoords_[5] = Vector3<T>(maxVertex_.x,minVertex_.y,maxVertex_.z);            
  vertexCoords_[6] = Vector3<T>(maxVertex_.x,maxVertex_.y,maxVertex_.z);            
  vertexCoords_[7] = Vector3<T>(minVertex_.x,maxVertex_.y,maxVertex_.z);            
  verticesAtBoundary_[0]   = 0;
  verticesAtBoundary_[1]   = 1;
  verticesAtBoundary_[2]   = 2;
  verticesAtBoundary_[3]   = 3;
  verticesAtBoundary_[4]   = 4;
  verticesAtBoundary_[5]   = 5;
  verticesAtBoundary_[6]   = 6;
  verticesAtBoundary_[7]   = 7;

  hexas_[0].hexaVertexIndices_[0]      = 0;
  hexas_[0].hexaVertexIndices_[1]      = 1;
  hexas_[0].hexaVertexIndices_[2]      = 2;
  hexas_[0].hexaVertexIndices_[3]      = 3;
  hexas_[0].hexaVertexIndices_[4]      = 4;
  hexas_[0].hexaVertexIndices_[5]      = 5;
  hexas_[0].hexaVertexIndices_[6]      = 6;
  hexas_[0].hexaVertexIndices_[7]      = 7;
  
}

template<class T,class Traits>
void UnstructuredGrid<T,Traits>::vertexOrderXYZ()
{

//  std::vector<sPerm> permArray;
//  
//  if(vertexOrder_ == NULL)
//    vertexOrder_ = new int[nvt_];
//
//  for(int i=0;i<nvt_;i++)
//  {
//    sPerm perm;
//    perm.vVec = vertexCoords_[i];
//    perm.index  = i;
//    permArray.push_back(perm);
//  }
//  
//  std::stable_sort(permArray.begin(),permArray.end(),coordz());
//  std::stable_sort(permArray.begin(),permArray.end(),coordy());
//  std::stable_sort(permArray.begin(),permArray.end(),coordx());
//  
//  for(int i=0;i<nvt_;i++)
//  {
//    vertexOrder_[i]=permArray[i].index;
//  }  
  
};

template<class T,class Traits>
void UnstructuredGrid<T,Traits>::genElAtVert()
{

  if(elementsAtVertexIdx_ == NULL)
	elementsAtVertexIdx_ = new int[nvt_+1];

  memset(elementsAtVertexIdx_,0,(nvt_+1)*sizeof(int));

  for(int i=0;i<nel_;i++)
  {
	for(int j=0;j<8;j++)
	{
	  int vindex=hexas_[i].hexaVertexIndices_[j]+1;
	  elementsAtVertexIdx_[vindex]++;
	}//end for
  }//end for
  
  elementsAtVertexIdx_[0]=0;
  for(int i=1;i<nvt_+1;i++)
  {
	elementsAtVertexIdx_[i]+=elementsAtVertexIdx_[i-1];
  }//end for

  int isize = elementsAtVertexIdx_[nvt_];
  elementsAtVertex_ = new int[isize];

  int *iaux = new int[nvt_+1];
  memcpy(iaux,elementsAtVertexIdx_,(nvt_+1)*sizeof(int));

  for(int i=0;i<nel_;i++)
  {
	for(int j=0;j<8;j++)
	{
	  int vindex=hexas_[i].hexaVertexIndices_[j];
	  elementsAtVertex_[iaux[vindex]]=i;
	  iaux[vindex]++;
	}//end for
  }//end for

  delete[] iaux;

};

template<class T,class Traits>
void UnstructuredGrid<T,Traits>::genNeighboursAtEl()
{

  typename UnstructuredGrid<T,Traits>::ConnectorList list(6*nel_);

  int nfaces=0;
  for(int i=0;i<nel_;i++)
  {
    //!=========================================================  
    //! first face
	  for(int k=0;k<4;k++)
	    list.pList[nfaces].idata[k] = hexas_[i].hexaVertexIndices_[k];

	  list.pList[nfaces].idata[4]   = i;

	  list.pList[nfaces].idata[5]   = 0;
    nfaces++;
    //!=========================================================  
    //! second face
    list.pList[nfaces].idata[0] = hexas_[i].hexaVertexIndices_[0];
    list.pList[nfaces].idata[1] = hexas_[i].hexaVertexIndices_[1];
    list.pList[nfaces].idata[2] = hexas_[i].hexaVertexIndices_[4];
    list.pList[nfaces].idata[3] = hexas_[i].hexaVertexIndices_[5];

	  list.pList[nfaces].idata[4]   = i;

	  list.pList[nfaces].idata[5]   = 1;
	  nfaces++;
    //!=========================================================  
    //! third face
    list.pList[nfaces].idata[0] = hexas_[i].hexaVertexIndices_[1];
    list.pList[nfaces].idata[1] = hexas_[i].hexaVertexIndices_[2];
    list.pList[nfaces].idata[2] = hexas_[i].hexaVertexIndices_[5];
    list.pList[nfaces].idata[3] = hexas_[i].hexaVertexIndices_[6];

	  list.pList[nfaces].idata[4]   = i;

	  list.pList[nfaces].idata[5]   = 2;
	  nfaces++;
    //!=========================================================  
    //! fourth face
    list.pList[nfaces].idata[0] = hexas_[i].hexaVertexIndices_[3];
    list.pList[nfaces].idata[1] = hexas_[i].hexaVertexIndices_[2];
    list.pList[nfaces].idata[2] = hexas_[i].hexaVertexIndices_[6];
    list.pList[nfaces].idata[3] = hexas_[i].hexaVertexIndices_[7];

	  list.pList[nfaces].idata[4]   = i;

	  list.pList[nfaces].idata[5]   = 3;
	  nfaces++;
    //!=========================================================  
    //! fifth face
    list.pList[nfaces].idata[0] = hexas_[i].hexaVertexIndices_[0];
    list.pList[nfaces].idata[1] = hexas_[i].hexaVertexIndices_[3];
    list.pList[nfaces].idata[2] = hexas_[i].hexaVertexIndices_[7];
    list.pList[nfaces].idata[3] = hexas_[i].hexaVertexIndices_[4];

	  list.pList[nfaces].idata[4]   = i;

	  list.pList[nfaces].idata[5]   = 4;
	  nfaces++;
    //!=========================================================  
    //! sixth face
	  for(int k=4;k<8;k++)
	    list.pList[nfaces].idata[k-4] = hexas_[i].hexaVertexIndices_[k];

	  list.pList[nfaces].idata[4]   = i;

	  list.pList[nfaces].idata[5]   = 5;
	  nfaces++;
  }//end for

  //sort the connector list
  list.sortdata();
  list.sort_list();

  //assign the neighbours at elements
  //traverse the connector list
  for(int k=1;k<6*nel_;k++)
  {
    int j=0;
    while(list.pList[k-1].idata[j]==list.pList[k].idata[j])
    {
      j++;
    }
    if(j==4)
    {
      hexas_[list.pList[k-1].idata[4]].hexaNeighborIndices_[list.pList[k-1].idata[5]]=list.pList[k].idata[4];
      hexas_[list.pList[k].idata[4]].hexaNeighborIndices_[list.pList[k].idata[5]]=list.pList[k-1].idata[4];
    }
  }//end for

}


template<class T,class Traits>
void UnstructuredGrid<T,Traits>::genEdgesAtEl()
{
  //the algorithm is as follows
  HexaEdge edges[12];
  int edge0 [2]={0,1};
  int edge1 [2]={1,2};
  int edge2 [2]={2,3};
  int edge3 [2]={3,0};
  int edge4 [2]={0,4};
  int edge5 [2]={1,5};
  int edge6 [2]={2,6};
  int edge7 [2]={3,7};
  int edge8 [2]={4,5};
  int edge9 [2]={5,6};
  int edge10[2]={6,7};
  int edge11[2]={7,4};
  int nedges=0;
  memcpy(edges[0].edgeVertexIndices_,edge0,2*sizeof(int));
  memcpy(edges[1].edgeVertexIndices_,edge1,2*sizeof(int));
  memcpy(edges[2].edgeVertexIndices_,edge2,2*sizeof(int));
  memcpy(edges[3].edgeVertexIndices_,edge3,2*sizeof(int));
  memcpy(edges[4].edgeVertexIndices_,edge4,2*sizeof(int));
  memcpy(edges[5].edgeVertexIndices_,edge5,2*sizeof(int));
  memcpy(edges[6].edgeVertexIndices_,edge6,2*sizeof(int));
  memcpy(edges[7].edgeVertexIndices_,edge7,2*sizeof(int));
  memcpy(edges[8].edgeVertexIndices_,edge8,2*sizeof(int));
  memcpy(edges[9].edgeVertexIndices_,edge9,2*sizeof(int));
  memcpy(edges[10].edgeVertexIndices_,edge10,2*sizeof(int));
  memcpy(edges[11].edgeVertexIndices_,edge11,2*sizeof(int));

  for(int i=0;i<nel_;i++)
  {
	
	for(int ied=0;ied<12;ied++)
	{
	  int iloc1 = edges[ied].edgeVertexIndices_[0];
	  int iloc2 = edges[ied].edgeVertexIndices_[1];

	  int iglob1 = hexas_[i].hexaVertexIndices_[iloc1];
	  int iglob2 = hexas_[i].hexaVertexIndices_[iloc2];

	  //findsmallestedge
	  int ismallest = findSmlstEl(iglob1,iglob2,i);

	  if(ismallest >= i)
	  {
		hexas_[i].hexaEdgeIndices_[ied]=nedges;
		nedges++;
	  }//end if
	  else
	  {
		for(int k=0;k<12;k++)
		{
		  int ed1 = hexas_[ismallest].hexaVertexIndices_[edges[k].edgeVertexIndices_[0]];
		  int ed2 = hexas_[ismallest].hexaVertexIndices_[edges[k].edgeVertexIndices_[1]];
		  if(((ed1==iglob1) && (ed2==iglob2)) || ((ed1==iglob2)&&(ed2==iglob1)))
		  {
			int foundEdge = hexas_[ismallest].hexaEdgeIndices_[k];
			hexas_[i].hexaEdgeIndices_[ied]=foundEdge;
			break;
		  }
		}//end for k
		
	  }//end else

	}//end for ied

  }//end for i

  nmt_ = nedges;

};

template<class T,class Traits>
void UnstructuredGrid<T,Traits>::genFacesAtEl()
{
	int iFace = 0;
	for(int i=0;i<nel_;i++)
	{
	  for(int j=0;j<6;j++)
	  {
		if((hexas_[i].hexaNeighborIndices_[j]==-1) 
		   || (hexas_[i].hexaNeighborIndices_[j]>i))
		{
		  hexas_[i].hexaFaceIndices_[j]=iFace;
		  iFace++;
		}
		else
		{
		  int jel = hexas_[i].hexaNeighborIndices_[j];
		  for(int k=0;k<6;k++)
		  {
			if(hexas_[jel].hexaNeighborIndices_[k]==i)
			{
			  hexas_[i].hexaFaceIndices_[j]=hexas_[jel].hexaFaceIndices_[k];
			  break;
			}
		  }//end k
		}//end else
	  }//end for
	}//end for

	nat_=iFace;

}//End GenFacesAtEl

template<class T,class Traits>
void UnstructuredGrid<T,Traits>::genVertAtEdg()
{

  int edgeindex[12][2] = 
  { {0,1},
	{1,2},
	{2,3},
	{3,0},
	{0,4},
	{1,5},
	{2,6},
	{3,7},
	{4,5},
	{5,6},
	{6,7},
	{7,4},
  };

  verticesAtEdge_ = new HexaEdge[this->nmt_];

  for(int i=0;i<nel_;i++)
  {
	  for(int j=0;j<12;j++)
	  {
	    int index = hexas_[i].hexaEdgeIndices_[j];
	    verticesAtEdge_[index].edgeVertexIndices_[0] = hexas_[i].hexaVertexIndices_[edgeindex[j][0]];
      verticesAtEdge_[index].edgeVertexIndices_[1] = hexas_[i].hexaVertexIndices_[edgeindex[j][1]];
	  }//end for
  }//end for

};

template<class T,class Traits>
void UnstructuredGrid<T,Traits>::genVertAtFac()
{
  int facesHex[6][4]=
  {
	{0,1,2,3},
	{0,4,5,1},
	{1,5,6,2},
	{2,6,7,3},
	{0,3,7,4},
	{4,7,6,5}
  };
  verticesAtFace_ = new HexaFace[this->nat_];
  for(int i=0;i<nel_;i++)
  {
	  for(int j=0;j<6;j++)
	  {
	    int faceindex=hexas_[i].hexaFaceIndices_[j];
	    for(int k=0;k<4;k++)
	    {
		    verticesAtFace_[faceindex].faceVertexIndices_[k]=
		    hexas_[i].hexaVertexIndices_[facesHex[j][k]];	  
	    }
	  }//end for j
  }//end for
};

  template<class T,class Traits>
  void UnstructuredGrid<T,Traits>::genVertexVertex()
  {
    m_VertexVertex = new VertexVertex[nvt_];
    for(int i=0;i<this->nmt_;i++)
    {
      int ia = verticesAtEdge_[i].edgeVertexIndices_[0];
      int ib = verticesAtEdge_[i].edgeVertexIndices_[1];
      int j=0;
      m_VertexVertex[ia].m_iVertInd[m_VertexVertex[ia].m_iNeighbors]=ib;
      m_VertexVertex[ia].m_iNeighbors++;
      m_VertexVertex[ib].m_iVertInd[m_VertexVertex[ib].m_iNeighbors]=ia;
      m_VertexVertex[ib].m_iNeighbors++;
    }
  };

  template<class T,class Traits>
  void UnstructuredGrid<T,Traits>::refine()
  {
    refineRaw();
    cleanExtended();
    refinementLevel_++;
  };

  template<class T,class Traits>
void UnstructuredGrid<T,Traits>::refineRaw()
{
  int iOffset=0;
  int iNVTnew = nvt_+nmt_+nat_+nel_;

  Vector3<T> *pVertexCoordsNew = new Vector3<T>[iNVTnew+1];

  int iNELnew=8*nel_;

  for(int i=0;i<nvt_;i++)
  {
    pVertexCoordsNew[i]=vertexCoords_[i];
  }//end for

  iOffset+=nvt_;

  for(int i=0;i<nmt_;i++)
  {
    int ivt1=this->verticesAtEdge_[i].edgeVertexIndices_[0];
    int ivt2=this->verticesAtEdge_[i].edgeVertexIndices_[1];

    Vector3<T> vMid = T(0.5) * (vertexCoords_[ivt1]+vertexCoords_[ivt2]);

    pVertexCoordsNew[iOffset+i]=vMid;

  }//end for

  iOffset+=nmt_;

  for(int i=0;i<nat_;i++)
  {
    Vector3<T> vMid(0,0,0);
    for(int j=0;j<4;j++)
    {
      vMid+=vertexCoords_[verticesAtFace_[i].faceVertexIndices_[j]];
    }//end for

    pVertexCoordsNew[iOffset+i]=vMid*(T)0.25;

  }//end for

  iOffset+=nat_;

  for(int i=0;i<nel_;i++)
  {

    Vector3<T> vMid(0,0,0);
    for(int j=0;j<8;j++)
    {
      vMid+=vertexCoords_[this->hexas_[i].hexaVertexIndices_[j]];
    }//end for j

    pVertexCoordsNew[iOffset+i]=vMid*(T)0.125;

  }//end for i

  iOffset=nel_;

  Hexa *pHexaNew = new Hexa[8*nel_];

  for(int i=0;i<nel_;i++)
  {

    int el[7];
    int edg2vert[12];
    int fac2vert[6];
    int midEl=nvt_+nmt_+nat_+i;
    for(int k=0;k<12;k++)edg2vert[k]=nvt_+hexas_[i].hexaEdgeIndices_[k];
    for(int k=0;k<6;k++)fac2vert[k]=nvt_+nmt_+hexas_[i].hexaFaceIndices_[k];
    for(int k=0;k<7;k++)el[k]=iOffset+k;

    // the 1st hexahedron
    pHexaNew[i].hexaVertexIndices_[0]= hexas_[i].hexaVertexIndices_[0];
    pHexaNew[i].hexaVertexIndices_[1]= edg2vert[0];
    pHexaNew[i].hexaVertexIndices_[2]= fac2vert[0];
    pHexaNew[i].hexaVertexIndices_[3]= edg2vert[3];

    pHexaNew[i].hexaVertexIndices_[4]= edg2vert[4];
    pHexaNew[i].hexaVertexIndices_[5]= fac2vert[1];
    pHexaNew[i].hexaVertexIndices_[6]= midEl;
    pHexaNew[i].hexaVertexIndices_[7]= fac2vert[4];

    // the 2nd new hexahedron
    pHexaNew[el[0]].hexaVertexIndices_[0]=hexas_[i].hexaVertexIndices_[1];
    pHexaNew[el[0]].hexaVertexIndices_[1]=edg2vert[1];
    pHexaNew[el[0]].hexaVertexIndices_[2]=fac2vert[0];
    pHexaNew[el[0]].hexaVertexIndices_[3]=edg2vert[0];

    pHexaNew[el[0]].hexaVertexIndices_[4]=edg2vert[5];
    pHexaNew[el[0]].hexaVertexIndices_[5]=fac2vert[2];
    pHexaNew[el[0]].hexaVertexIndices_[6]=midEl;
    pHexaNew[el[0]].hexaVertexIndices_[7]=fac2vert[1];

    // the 3rd new hexahedron
    pHexaNew[el[1]].hexaVertexIndices_[0]=hexas_[i].hexaVertexIndices_[2];
    pHexaNew[el[1]].hexaVertexIndices_[1]=edg2vert[2];
    pHexaNew[el[1]].hexaVertexIndices_[2]=fac2vert[0];
    pHexaNew[el[1]].hexaVertexIndices_[3]=edg2vert[1];

    pHexaNew[el[1]].hexaVertexIndices_[4]=edg2vert[6];
    pHexaNew[el[1]].hexaVertexIndices_[5]=fac2vert[3];
    pHexaNew[el[1]].hexaVertexIndices_[6]=midEl;
    pHexaNew[el[1]].hexaVertexIndices_[7]=fac2vert[2];

    // the 4th new hexahedron
    pHexaNew[el[2]].hexaVertexIndices_[0]=hexas_[i].hexaVertexIndices_[3];
    pHexaNew[el[2]].hexaVertexIndices_[1]=edg2vert[3];
    pHexaNew[el[2]].hexaVertexIndices_[2]=fac2vert[0];
    pHexaNew[el[2]].hexaVertexIndices_[3]=edg2vert[2];

    pHexaNew[el[2]].hexaVertexIndices_[4]=edg2vert[7];
    pHexaNew[el[2]].hexaVertexIndices_[5]=fac2vert[4];
    pHexaNew[el[2]].hexaVertexIndices_[6]=midEl;
    pHexaNew[el[2]].hexaVertexIndices_[7]=fac2vert[3];

    // the 5th new hexahedron
    pHexaNew[el[3]].hexaVertexIndices_[0]=hexas_[i].hexaVertexIndices_[4];
    pHexaNew[el[3]].hexaVertexIndices_[1]=edg2vert[11];
    pHexaNew[el[3]].hexaVertexIndices_[2]=fac2vert[5];
    pHexaNew[el[3]].hexaVertexIndices_[3]=edg2vert[8];

    pHexaNew[el[3]].hexaVertexIndices_[4]=edg2vert[4];
    pHexaNew[el[3]].hexaVertexIndices_[5]=fac2vert[4];
    pHexaNew[el[3]].hexaVertexIndices_[6]=midEl;
    pHexaNew[el[3]].hexaVertexIndices_[7]=fac2vert[1];

    // the 6th new hexahedron
    pHexaNew[el[4]].hexaVertexIndices_[0]=hexas_[i].hexaVertexIndices_[5];
    pHexaNew[el[4]].hexaVertexIndices_[1]=edg2vert[8];
    pHexaNew[el[4]].hexaVertexIndices_[2]=fac2vert[5];
    pHexaNew[el[4]].hexaVertexIndices_[3]=edg2vert[9];

    pHexaNew[el[4]].hexaVertexIndices_[4]=edg2vert[5];
    pHexaNew[el[4]].hexaVertexIndices_[5]=fac2vert[1];
    pHexaNew[el[4]].hexaVertexIndices_[6]=midEl;
    pHexaNew[el[4]].hexaVertexIndices_[7]=fac2vert[2];

    // the 7th new hexahedron
    pHexaNew[el[5]].hexaVertexIndices_[0]=hexas_[i].hexaVertexIndices_[6];
    pHexaNew[el[5]].hexaVertexIndices_[1]=edg2vert[9];
    pHexaNew[el[5]].hexaVertexIndices_[2]=fac2vert[5];
    pHexaNew[el[5]].hexaVertexIndices_[3]=edg2vert[10];

    pHexaNew[el[5]].hexaVertexIndices_[4]=edg2vert[6];
    pHexaNew[el[5]].hexaVertexIndices_[5]=fac2vert[2];
    pHexaNew[el[5]].hexaVertexIndices_[6]=midEl;
    pHexaNew[el[5]].hexaVertexIndices_[7]=fac2vert[3];

    // the 8th new hexahedron
    pHexaNew[el[6]].hexaVertexIndices_[0]=hexas_[i].hexaVertexIndices_[7];
    pHexaNew[el[6]].hexaVertexIndices_[1]=edg2vert[10];
    pHexaNew[el[6]].hexaVertexIndices_[2]=fac2vert[5];
    pHexaNew[el[6]].hexaVertexIndices_[3]=edg2vert[11];

    pHexaNew[el[6]].hexaVertexIndices_[4]=edg2vert[7];
    pHexaNew[el[6]].hexaVertexIndices_[5]=fac2vert[3];
    pHexaNew[el[6]].hexaVertexIndices_[6]=midEl;
    pHexaNew[el[6]].hexaVertexIndices_[7]=fac2vert[4];

    iOffset+=7;

  }//end for

  int *piVertAtBdrNew = new int[iNVTnew];
  for(int i=0;i<nvt_;i++)
  {
    piVertAtBdrNew[i]=verticesAtBoundary_[i];
  }

  //assign new vertex properties for edge midpoints
  for(int i=0;i<nmt_;i++)
  {
    int ivt1=this->verticesAtEdge_[i].edgeVertexIndices_[0];
    int ivt2=this->verticesAtEdge_[i].edgeVertexIndices_[1];

    if(verticesAtBoundary_[ivt1]==1 && verticesAtBoundary_[ivt2]==1)
    {
      piVertAtBdrNew[nvt_+i]=1;
    }
    else
    {
      piVertAtBdrNew[nvt_+i]=0;
    }
  }//end for  

  //assign new vertex properties for face midpoints
  for(int i=0;i<nat_;i++)
  {
    int ivt1 = verticesAtFace_[i].faceVertexIndices_[0];
    int ivt2 = verticesAtFace_[i].faceVertexIndices_[1];
    int ivt3 = verticesAtFace_[i].faceVertexIndices_[2];
    int ivt4 = verticesAtFace_[i].faceVertexIndices_[3];

    if(verticesAtBoundary_[ivt1]==1 && verticesAtBoundary_[ivt2]==1 && verticesAtBoundary_[ivt3]==1 && verticesAtBoundary_[ivt4]==1)
    {
      piVertAtBdrNew[nvt_+nmt_+i]=1;            
    }
    else
    {
      piVertAtBdrNew[nvt_+nmt_+i]=0;      
    }
  }//end for

  for(int i=0;i<nel_;i++)
  {
    piVertAtBdrNew[nvt_+nmt_+nat_+i]=0;            
  }//end for    

  delete[] vertexCoords_;
  vertexCoords_ = pVertexCoordsNew;


  verticesAtHexaLev_.push_back(hexas_);
  hexas_=pHexaNew;

  nvt_=iNVTnew;

  nel_=iNELnew;

  delete[] m_myTraits;
  m_myTraits = new Traits[iNVTnew];

  verticesAtBoundaryLev_.push_back(verticesAtBoundary_);
  verticesAtBoundary_ = piVertAtBdrNew;

};

  template<class T,class Traits>
void UnstructuredGrid<T,Traits>::vertAtBdr()
{

  int *verticesTemp=new int[nvt_];

  for(int i=0;i<nvt_;i++)
  {
    verticesTemp[i]=verticesAtBoundary_[i];
    verticesAtBoundary_[i]=0;
  }

  for(int i=0;i<nel_;i++)
  {
    for(int j=0;j<6;j++)
    {
      if(hexas_[i].hexaNeighborIndices_[j]==-1)
      {
        //verticesAtBoundary_[nvt_+nmt_+nat_+i]=0;            
        //std::cout<<"Found boundary face... "<<std::endl;      
        int faceindex=hexas_[i].hexaFaceIndices_[j];      
        for(int k=0;k<4;k++)
        {
          if(verticesTemp[verticesAtFace_[faceindex].faceVertexIndices_[k]]!=0)
            verticesAtBoundary_[verticesAtFace_[faceindex].faceVertexIndices_[k]]=1;                    
        }
      }
    }
  }//end for  

  delete[] verticesTemp;

}//End VertAtBdr

  template<class T, class Traits>
void UnstructuredGrid<T, Traits>::pertubeMesh()
{
  T edgeLength = (vertexCoords_[verticesAtEdge_[0].edgeVertexIndices_[0]] -
      vertexCoords_[verticesAtEdge_[0].edgeVertexIndices_[1]]).mag();

  for (int i = 0; i < nvt_; i++)
  {
    if (verticesAtBoundary_[i])
      continue;


    Vector3<T> dir(frand(), frand(), frand());
    dir.Normalize();
    dir *= 0.2*edgeLength;
    //vertexCoords_[i].x += 0.1*edgeLength * Vector3<T>(1.0, 0.0, 0.0);
    //vertexCoords_[i].x += 0.2*edgeLength;
    vertexCoords_[i] += dir;

  }
}

  template<class T,class Traits>
void UnstructuredGrid<T,Traits>::initStdMesh()
{
#ifdef WINDOWS
  CTimer timer;
  timer.Start();
#endif
  genElAtVert();
#ifdef WINDOWS
  std::cout<<"Time for routine GenElAtVert(): "<<timer.Elapsed()<<std::endl;
  timer.Stop();
  timer.Start();
#endif
  genNeighboursAtEl();
#ifdef WINDOWS
  std::cout<<"Time for routine genNeighboursAtEl(): "<<timer.Elapsed()<<std::endl;
  timer.Stop();
  timer.Start();
#endif
  genEdgesAtEl();
#ifdef WINDOWS
  std::cout<<"Time for routine GenEdgesAtEl(): "<<timer.Elapsed()<<std::endl;
  timer.Stop();
  timer.Start();
#endif
  genFacesAtEl();
#ifdef WINDOWS
  std::cout<<"Time for routine GenFacesAtEl(): "<<timer.Elapsed()<<std::endl;
  timer.Stop();
  timer.Start();
#endif
  genVertAtEdg();
#ifdef WINDOWS
  std::cout<<"Time for routine GenVertAtEdg(): "<<timer.Elapsed()<<std::endl;
  timer.Stop();
  timer.Start();
#endif
  genVertAtFac();
#ifdef WINDOWS
  std::cout<<"Time for routine GenVertAtFac(): "<<timer.Elapsed()<<std::endl;
  timer.Stop();
  timer.Start();
#endif
  genVertexVertex();

  nvtLev_.push_back(nvt_);
  nmtLev_.push_back(nmt_);
  natLev_.push_back(nat_);
  nelLev_.push_back(nel_);

  verticesAtEdgeLev_.push_back(verticesAtEdge_);
  verticesAtFaceLev_.push_back(verticesAtFace_);

  if(refinementLevel_==1)  
    vertAtBdr();

};

  template<class T,class Traits>
void UnstructuredGrid<T,Traits>::cleanExtended()
{

  if(this->elementsAtVertex_)
  {
    delete[] elementsAtVertex_;
    elementsAtVertex_=NULL;
  }

  if(this->elementsAtVertexIdx_)
  {
    delete[] elementsAtVertexIdx_;
    elementsAtVertexIdx_=NULL;
  }

  if(this->verticesAtEdge_)
  {
    //delete[] verticesAtEdge_;
    verticesAtEdge_=NULL;
  }

  if(this->verticesAtFace_)
  {
    verticesAtFace_=NULL;
  }

};

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class UnstructuredGrid<double,DTraits>;
template class UnstructuredGrid<float,DTraits>;

//----------------------------------------------------------------------------

}
