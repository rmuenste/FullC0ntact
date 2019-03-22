
#ifdef WIN32
#pragma once
#endif

#ifndef _UNSTRUCTUREDGRID_H
#define _UNSTRUCTUREDGRID_H

//===================================================
//                    DEFINITIONS
//===================================================

//===================================================
//                    INCLUDES
//===================================================
#include <iostream>
#include <algorithm>
#include <vector3.h>
#include <cstring>
#include <fstream>
#include <vector>
#include <aabb3.h>
#include <iterators.hpp>
#include <tetrahedron.hpp>
#ifdef WINDOWS
#include <Timer.h>
#endif

//===================================================
//                      Class
//===================================================

namespace i3d {

///@cond HIDDEN_SYMBOLS
typedef struct
{
  Vector3<Real> vVec;
  int            index;
}sPerm;  
//
///@cond HIDDEN_SYMBOLS
class ParFileInfo
{
  public:
  std::string name_;
  std::string expression_;
  std::string boundaryType_;
  std::vector<int> nodes_;
  std::vector<int> glob2Loc_;
  std::vector<HexaFace> verticesAtFace_;
  std::vector<int> faces_;
  unsigned size_;
};  
  
/**
* @brief Example class for user-defined vertex traits
*
*/
class DTraits
{
public:
  Real distance;
  Real dist2;
  Real mass_;
  Real t_;
  int    iTag;
  int    iX;
  Vector3<Real> vRef;
  Vector3<Real> vNormal;  
  Vector3<Real> vel_;
  Vector3<Real> force_;
  Vector3<Real> pos_old_;
  bool fixed_;
  bool flagella_;

  DTraits() : 
    distance(0.0f), dist2(0.0f),mass_(0.5),t_(0.0),
    iTag(0),iX(0), vRef(0, 0, 0), vNormal(0, 0, 0),
    vel_(0, 0, 0), force_(0, 0, 0), pos_old_(0, 0, 0),
    fixed_(false), flagella_(false)
  {

  }

};

class FTraits
{
public:
  float distance;
  float dist2;
  int   iTag;
  int   iX;
  Vector3<float> vRef;
  Vector3<float> vNormal;  
};

class DistQueueEntry
{
public:

  Vector3<Real> *vPoint;
  DTraits *pTraits;
  int     iVert;

};


/**
* @brief An unstructured mesh class in 3d
*
* An unstructured mesh class in 3d that consists of vertices, edges, faces and hexahedrons.
*  
*
*/
template<class T,class VTraits = DTraits>
class UnstructuredGrid
{
public:
  
  UnstructuredGrid(void);
  
  ~UnstructuredGrid(void);

  std::vector<ParFileInfo> *parInfo_;
  
  Vector3<T> *vertexCoords_;

  // Vertices Adjacent to an Element.
  // Handle to h_IverticesAtElement=array [1..NVE,1..NEL] of integer
  // For each element the node numbers of the corner-vertices
  // in mathematically positive sense.
  // On pure triangular meshes, there is NVE=3. On mixed or pure quad
  // meshes, there is NVE=4. In this case, there is 
  // IverticesAtElement(4,.)=0 for a triangle in a quad mesh.
  // This is a handle to the old KVERT array.
  Hexa *hexas_;

  // Handle to 
  //       p_IverticesAtBoundary = array [1..NVBD] of integer.
  // This array contains a list of all vertices on the (real) boundary
  //in mathematically positive sense.
  // The boundary vertices of boundary component i are saved at
  //        p_IboundaryCpIdx(i)..p_IboundaryCpIdx(i+1)-1.
  // This is the old KVBD array.
  int *verticesAtBoundary_;

  //      p_IfacesAtBoundary = array [1..NMBD] of integer.
  //This array contains a list of all edges on the (real) boundary
  //with increasing number.
  //The boundary edges of boundary component i are saved at
  //       p_IboundaryCpFacesIdx(i)..p_IboundaryCpFacesIdx(i+1)-1.
  //int *facesAtBoundary_;
  std::vector<int> facesAtBoundary_;

  // Elements Adjacent to the boundary. 
  // Handle to 
  //      p_IelementsAtBoundary = array [1..NVBD] of integer.
  // This array contains a list of all elements on the (real) boundary
  // in mathematically positive sense.
  // p_IelementsAtBoundary(i) is the element adjacent to edge
  // h_IedgesAtBoundary - therefore one element number might appear
  // more than once in this array!
  // The boundary elements of boundary component i are saved at
  //        p_IboundaryCpIdx(i)..p_IboundaryCpIdx(i+1)-1.
  // This is the old KEBD array.
  int *elementsAtBoundary_;

  // Vertices Adjacent to a face.
  // Handle to 
  //       p_IverticesAtFace = array [1..NVA,1..NAT] of integer
  // For each face, the numbers of the vertices adjacent to that
  // face. Ordered in mathematically positive sense when looking
  // from the midpoint of the adjacent element with the smaller
  // element number.
  // On pure tetrahedral meshes, there is NVA=3. On mixed or pure 
  // hexahedral meshes, there is NVA=4. In this case, there is 
  // IverticesAtFace(4,.)=0 for a tetrahedral in a hexahedral mesh.
  HexaFace *verticesAtFace_;

  // Vertices Adjacent to an Edge. 
  // Handle to 
  //       p_IverticesAtEdge = array [1..2,1..NMT]
  // The numbers of the two vertices adjacent to an edge IMT. 
  HexaEdge *verticesAtEdge_;

  // p_IneighboursAtElement = array [1..TRIA_MAXNME2D,1..NEL] of integer
  // For each element, the numbers of adjacent elements
  // in mathematically positive sense, meeting the element in an edge.
  // p_RneighbourElement(IEL)\%Ineighbours(.) describes the elements adjacent 
  // to IEL along the edges (p_RedgesOnElement(IEL)\%Iedges(.)-NVT).
  // This is the old KADJ array.
  Hexa *neighborsAtElement_;
  
  VertexVertex *m_VertexVertex;

  VTraits *m_myTraits;


  /**
   * Indices of the elements adjacent to a vertex
   */
  int *elementsAtVertexIdx_;

  /**
   * Indices of the elements adjacent to a vertex
   */  
  int *elementsAtVertex_;
  
  /**
   * 
   */
  int *vertexOrder_;

  // the total number of vertices
  int nvt_;

  // the total number of edges
  int nmt_;

  // the total number of faces
  int nat_;

  // the total number of elements
  int nel_;
  
  std::vector<int> nvtLev_;
  std::vector<int> nmtLev_;
  std::vector<int> natLev_;
  std::vector<int> nelLev_;
  std::vector<HexaEdge*> verticesAtEdgeLev_;
  std::vector<HexaFace*> verticesAtFaceLev_;
  std::vector<Hexa*> verticesAtHexaLev_;
  std::vector<int*> verticesAtBoundaryLev_;

  std::vector<T> elemVol_;
  std::vector<T> elementDist_;
  std::vector<Vector3<T>> faceNormals_; 

  T vol_;

  int refinementLevel_;
  
  Vector3<T> minVertex_;
  Vector3<T> maxVertex_;  
  
  Vector3<T>& Vertex(int i) {return vertexCoords_[i];};
  const Vector3<T>& Vertex(int i) const {return vertexCoords_[i];};

  VTraits& VertexTrait(int i) {return m_myTraits[i];};
  const VTraits& VertexTrait(int i) const {return m_myTraits[i];};

  /**
   * Computes the vertices at the boundary of the mesh by 
   * examining the boundary faces.
   */
  void vertAtBdr();

  /**
   * Computes the faces at the boundary of the mesh by 
   * examining the boundary faces.
   */
  void facesAtBdr();
  
  /**
   * Constructs a mesh from the unit cube
   */
  void initUnitCube();

  /**
   * Calculate the volume for each element
   */
  void calcVol();

  /**
   * Calculate the volume for each element
   */
  T elemVol(int idx);

  /**
   * Return whether the point is inside the mesh
   */
  bool pointInside(const Vector3<T> &query);

  /**
   * Return whether the point is inside the hexa
   */
  bool pointInsideHexa(int hIdx, const Vector3<T> &query);

  bool pointInsideHexaDebug(int hIdx, const Vector3<T> &query);

  /**
   * Return whether the point is inside the hexa
   */
  bool pointInsideTetra(const Vector3<T> &query,
                        const Vector3<T> &a, 
                        const Vector3<T> &b,
                        const Vector3<T> &c,
                        const Vector3<T> &d,
                        bool debug=false);

  /**
   * Return whether the point is inside the hexa
   */
  bool sameSide(const Vector3<T> &query,
                        const Vector3<T> &a, 
                        const Vector3<T> &b,
                        const Vector3<T> &c,
                        const Vector3<T> &d,
                        bool debug=false);
  
  /**
   * Contructs a mesh from a mesh stored in a file. The file format 
   * has to be the devisor .tri format
   */
  void initMeshFromFile(const char *strFileName);

  /**
   * Apply a decimation operation to the mesh
   */
  void decimate();
  
  /**
   * Computes the standard set of connectivity arrays
   */
  void initStdMesh();

  void pertubeMesh();

  void vertexOrderXYZ();

  AABB3<T> getAABB()
  {
    return AABB3<T>(minVertex_,maxVertex_);
  };
  
  /**
   * Applies a regular refinement strategy on the mesh
   */
  void refine();
  
  /**
   * Initializes a grid for a box, specified by 2 points in space
   * @param xmin Minimal x-coordinate
   * @param ymin Minimal y-coordinate
   * @param zmin Minimal z-coordinate
   * @param xmax Maximal x-coordinate
   * @param ymax Maximal y-coordinate
   * @param zmaz Maximal z-coordinate
   */
  void initCube(T xmin, T ymin, T zmin, T xmax, T ymax, T zmax);
  
  /**
   * Initializes a grid from an aabb
   * @param aabb an axis-aligned bounding box
   */
  void initCubeFromAABB(const AABB3<T> &aabb);  


  /**
   * @brief An ElemVertIter iterates over elements attached to a vertex
   *
   */      
  class ElemVertIter
  {
    public:

      typedef Hexa  value_type;
      typedef Hexa* pointer;
      typedef Hexa& reference;
      ElemVertIter(int *curpos = nullptr,UnstructuredGrid<T,VTraits> *grid=nullptr) : _curpos(curpos),m_pGrid(grid) {};

      reference operator*() {return m_pGrid->hexas_[*_curpos];}

      ElemVertIter& operator++()
      {
        _curpos=_curpos+1;
        return *this;
      }

      int Get(){return *_curpos;};

      ElemVertIter operator++(int)
      {
        ElemVertIter old_it = *this;
        ++(*this);
        return old_it;
      }

      bool operator !=(ElemVertIter rhs){return _curpos!=rhs._curpos;};

    protected:
      int* _curpos;
      UnstructuredGrid<T,VTraits> *m_pGrid;
  };

  //----------------------------------------------------------------------------	

  //----------------------------------------------------------------------------	

  /**
   * @brief An VertElemIter iterates over the vertices of a Hexa
   *
   */    
  class VertElemIter
  {
    public:

      typedef Vector3<T>  value_type;
      typedef Vector3<T>* pointer;
      typedef Vector3<T>& reference;
      VertElemIter(int curpos = 0,UnstructuredGrid<T,VTraits> *grid=NULL,Hexa *pHexa=NULL,int iHexa=0) : _curpos(curpos),m_pGrid(grid),m_pHexa(pHexa),m_iHexa(iHexa) {};

      reference operator*() {return  m_pGrid->vertexCoords_[m_pGrid->hexas_[m_iHexa].hexaVertexIndices_[_curpos]];};

      VertElemIter& operator++()
      {
        _curpos=_curpos+1;
        return *this;
      }

      int GetInt(){return m_pGrid->hexas_[m_iHexa].hexaVertexIndices_[_curpos];};

      VertElemIter operator++(int)
      {
        VertElemIter old_it = *this;
        ++(*this);
        return old_it;
      }

      bool operator !=(VertElemIter rhs){return _curpos!=rhs._curpos;};

    protected:
      int _curpos;
      UnstructuredGrid<T,VTraits> *m_pGrid;
      Hexa *m_pHexa;
      int m_iHexa;
  };
  //----------------------------------------------------------------------------	

  typename UnstructuredGrid<T,VTraits>::VertElemIter VertElemBegin(Hexa* pHexa);
  typename UnstructuredGrid<T,VTraits>::VertElemIter VertElemEnd(Hexa* pHexa);

  ElementIter elem_begin();
  ElementIter elem_end();

  HFaceIter faces_begin();
  HFaceIter faces_end();

  VertexVertexIter VertexVertexBegin(VertexIter<T> vIter);
  VertexVertexIter VertexVertexEnd(VertexIter<T> vIter);

  VertexVertexIter VertexVertexBegin(int iVert);
  VertexVertexIter VertexVertexEnd(int iVert);


  //the function returns an EdgeIter that points to the first edge
  EdgeIter edge_begin();
  //the function returns an EdgeIter that points to end of the edge array
  EdgeIter edge_end();

  //the function returns a VertexIter that points to the first vertex
  VertexIter<T> vertices_begin();
  VertexIter<T> vertices_end();

  //ElemVertIter GetElemVertIter(VertexIter vIter);
  typename UnstructuredGrid<T,VTraits>::ElemVertIter begin(VertexIter<T> vIter);
  typename UnstructuredGrid<T,VTraits>::ElemVertIter end(VertexIter<T> vIter);
  typename UnstructuredGrid<T,VTraits>::ElemVertIter begin(int vid);
  typename UnstructuredGrid<T,VTraits>::ElemVertIter end(int vid);


  friend class VertexVertexIter;
  friend class ElemVertIter;
  friend class VertexIter<T>;
  friend class VertElemIter;
  friend class ElementIter;
  friend class EdgeIter;	
  friend class HFaceIter;



private:
  void refineRaw();
  void genEdgesAtEl();
  void genElAtVert();
  void genNeighboursAtEl();
  void genFacesAtEl();
  void genVertAtEdg();
  void genVertAtFac();
  void genVertexVertex();
  void cleanExtended();

  inline float frand()
  {

    float r = rand() / (float)RAND_MAX;
    r -= 0.5f;
    return r;
  }

  int  findSmlstEl(int ivt1, int ivt2, int iel)
  {
    int iSmlstEl=iel;
    int iStart1,iStart2;
    int iEnd1,iEnd2;
    int iel1,iel2;
    iStart1=elementsAtVertexIdx_[ivt1];
    iStart2=elementsAtVertexIdx_[ivt2];
    iEnd1=elementsAtVertexIdx_[ivt1+1]-1;
    iEnd2=elementsAtVertexIdx_[ivt2+1]-1;

    for(int i=iStart1;i<iEnd1;i++)
    {
      iel1  = elementsAtVertex_[i];

      for(int j=iStart2;j<iEnd2;j++)
      {
        iel2 = elementsAtVertex_[j];

        if((iel2==iel1) && (iel2 < iSmlstEl))
        {
          iSmlstEl=iel2;
        }//end if

      }//end for

    }//end for

    return iSmlstEl;
  };

  ///@cond HIDDEN_SYMBOLS
  class ConnectorList
  {

    class Connector
    {
      public:
        Connector(){};
        ~Connector(){};

        int idata[6];
    };

    public:

    ConnectorList(int iSize)
    {
      m_iSize=iSize;
      pList = new Connector[iSize];
    };//

    ~ConnectorList(void)
    {
      delete[] pList;
    };//

    template <int n>
      struct lt
      {
        bool operator()(const  Connector &elem1, const  Connector &elem2)
        {
          return elem1.idata[n] < elem2.idata[n];
        }
      };

    int m_iSize;
    Connector *pList;

    void sort_list()
    {
      std::stable_sort(pList,pList+m_iSize,lt<3>());
      std::stable_sort(pList,pList+m_iSize,lt<2>());
      std::stable_sort(pList,pList+m_iSize,lt<1>());
      std::stable_sort(pList,pList+m_iSize,lt<0>());
    };

    void sortdata()
    {
      for(int i=0;i<m_iSize;i++)
      {
        std::sort(pList[i].idata,pList[i].idata+4);
      }//end for
    }

  };
  ///@cond HIDDEN_SYMBOLS
};

  template<class T, class Traits>
inline HFaceIter UnstructuredGrid<T, Traits>::faces_begin()
{
  return HFaceIter(verticesAtFace_, 0);
};//end 

  template<class T, class Traits>
inline HFaceIter UnstructuredGrid<T, Traits>::faces_end()
{
  return HFaceIter(verticesAtFace_ + nat_, nat_);
};//end 

  template<class T,class Traits>
inline ElementIter UnstructuredGrid<T,Traits>::elem_begin()
{  
  return ElementIter(hexas_,0);
};//end 

  template<class T,class Traits>
inline ElementIter UnstructuredGrid<T,Traits>::elem_end()
{
  return ElementIter(hexas_+nel_,nel_);
};//end 

  template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::VertElemIter UnstructuredGrid<T,Traits>::VertElemBegin(Hexa* pHexa)
{
  int diff = (pHexa-hexas_);
  return VertElemIter(0,this,pHexa,diff);
};//end 

  template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::VertElemIter UnstructuredGrid<T,Traits>::VertElemEnd(Hexa* pHexa)
{
  int diff = (pHexa-hexas_);
  return VertElemIter(8,this,pHexa,diff);
};//end

  template<class T,class Traits>
inline VertexVertexIter UnstructuredGrid<T,Traits>::VertexVertexBegin(VertexIter<T> vIter)
{
  int diff = (vIter.Get()-vertexCoords_);

  return VertexVertexIter(0,&m_VertexVertex[diff],diff);
};//end 

  template<class T,class Traits>
inline VertexVertexIter UnstructuredGrid<T,Traits>::VertexVertexEnd(VertexIter<T> vIter)
{
  int diff = (vIter.Get()-vertexCoords_);
  int iEnd = m_VertexVertex[diff].m_iNeighbors;
  return VertexVertexIter(iEnd,&m_VertexVertex[diff],diff);
};//end 

  template<class T,class Traits>
inline VertexVertexIter UnstructuredGrid<T,Traits>::VertexVertexBegin(int iVert)
{
  return VertexVertexIter(0,&m_VertexVertex[iVert],iVert);
};//end 

  template<class T,class Traits>
inline VertexVertexIter UnstructuredGrid<T,Traits>::VertexVertexEnd(int iVert)
{
  int iEnd = m_VertexVertex[iVert].m_iNeighbors;
  return VertexVertexIter(iEnd,&m_VertexVertex[iEnd],iVert);
};//end 

template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::ElemVertIter UnstructuredGrid<T,Traits>::begin(VertexIter<T> vIter)
{
  int diff = (vIter.Get()-vertexCoords_);
  return ElemVertIter(&elementsAtVertex_[elementsAtVertexIdx_[diff]],this);
};//end 

  template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::ElemVertIter UnstructuredGrid<T,Traits>::end(VertexIter<T> vIter)
{
  int diff = (vIter.Get()-vertexCoords_);
  return ElemVertIter(&elementsAtVertex_[elementsAtVertexIdx_[diff+1]],this);
};//end 
//--------------------------------------------
template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::ElemVertIter UnstructuredGrid<T,Traits>::begin(int vid)
{
  return ElemVertIter(&elementsAtVertex_[elementsAtVertexIdx_[vid]],this);
};//end 

template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::ElemVertIter UnstructuredGrid<T,Traits>::end(int vid)
{
  return ElemVertIter(&elementsAtVertex_[elementsAtVertexIdx_[vid+1]],this);
};//end 
//--------------------------------------------
  template<class T,class Traits>
inline EdgeIter UnstructuredGrid<T,Traits>::edge_begin()
{
  return EdgeIter(verticesAtEdge_,0);
};//end 

  template<class T,class Traits>
inline EdgeIter UnstructuredGrid<T,Traits>::edge_end()
{
  return EdgeIter(verticesAtEdge_ + (nmt_));
};//end 

  template<class T,class Traits>
inline VertexIter<T> UnstructuredGrid<T,Traits>::vertices_begin()
{
  return VertexIter<T>(vertexCoords_,0);
};//end 

  template<class T,class Traits>
inline VertexIter<T> UnstructuredGrid<T,Traits>::vertices_end()
{
  return VertexIter<T>(vertexCoords_ + (nvt_),nvt_);
};//end

typedef UnstructuredGrid<Real, DTraits> CUnstrGrid;
typedef UnstructuredGrid<Real, DTraits> CUnstrGridr;
typedef UnstructuredGrid<float, FTraits> UnstrGridf;

}

#endif
