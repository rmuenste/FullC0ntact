
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
  
/**
* @brief Example class for user-defined vertex traits
*
*/
class DTraits
{
public:
  Real distance;
  Real dist2;
  int    iTag;
  int    iX;
  Vector3<Real> vRef;
  Vector3<Real> vNormal;  
};

class FTraits
{
public:
  float distance;
  float dist2;
  int    iTag;
  int    iX;
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
* @brief Class that represents a hexahedron
*
*/
class Hexa
{
public:
  Hexa()
  {
    memset(hexaNeighborIndices_,-1,6*sizeof(int));
    memset(hexaFaceIndices_,-1,6*sizeof(int));
    memset(hexaVertexIndices_,-1,8*sizeof(int));
    memset(hexaEdgeIndices_,-1,12*sizeof(int));
  };

  Hexa& operator=(const Hexa &hex)
  {
    memcpy(hexaVertexIndices_,hex.hexaVertexIndices_,8*sizeof(int));
    memcpy(hexaEdgeIndices_,hex.hexaEdgeIndices_,12*sizeof(int));
    memcpy(hexaFaceIndices_,hex.hexaFaceIndices_,6*sizeof(int));
    memcpy(hexaNeighborIndices_,hex.hexaNeighborIndices_,6*sizeof(int));
    return *this;
  };

  Hexa(const Hexa &hex)
  {
	memcpy(hexaVertexIndices_,hex.hexaVertexIndices_,8*sizeof(int));
	memcpy(hexaEdgeIndices_,hex.hexaEdgeIndices_,12*sizeof(int));
	memcpy(hexaFaceIndices_,hex.hexaFaceIndices_,6*sizeof(int));
	memcpy(hexaNeighborIndices_,hex.hexaNeighborIndices_,6*sizeof(int));
  };

  /**
   * The array maps the indices of the 8 local nodes to the
   * the global vertex array, so that the vertex coordinates
   * can be accessed.
   **/
  int hexaVertexIndices_[8];
  /**
   * The array maps the indices of the vertices of the local 12 edges 
   * to the global vertex array, so that the coordinates of the edge
   * vertices can be accessed
   */
  int hexaEdgeIndices_[12];
  /**
   * The array maps the local vertex indices of the 6 faces to
   * the global vertex array, so that the coordinates of the vertices
   * can be accessed
   */
  int hexaFaceIndices_[6];
  /**
   * The array stores the element id of the neighbouring hexas
   * at the 6 faces of the hexa. If there is no neighbour at a
   * face the array stores a -1 at this position.
   */
  int hexaNeighborIndices_[6];
  
};

/**
* @brief Class that represents a quadrilateral face in 3d
*
*/
class HexaFace
{
public:
  int faceVertexIndices_[4];
};

class CVertexVertex
{
public:
  CVertexVertex()
  {
    memset(m_iVertInd,-1,6*sizeof(int));
    m_iNeighbors = 0;
  };

  int m_iVertInd[6];
  int m_iNeighbors;
};

/**
* @brief Class that an edge between to points in 3d
*
*/
class HexaEdge
{
public:
  int edgeVertexIndices_[2];
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
  int *facesAtBoundary_;

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
  
  CVertexVertex *m_VertexVertex;

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
   * Constructs a mesh from the unit cube
   */
  void initUnitCube();
  
  
  /**
   * Contructs a mesh from a mesh stored in a file. The file format 
   * has to be the devisor .tri format
   */
  void initMeshFromFile(const char *strFileName);
  
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

  //----------------------------------------------------------------------------
  //Class VertexIter: the vertex iterator iterates over all vertices of the mesh
  /**
  * @brief A VertexIter iterates over vertices of a CUnstructuredGrid
  *
  */  
  class VertexIter
  {
  public:
    
    typedef Vector3<T>  value_type;
    typedef Vector3<T>* pointer;
    typedef Vector3<T>& reference;
    VertexIter(Vector3<T>* curpos = NULL, int ipos=0) : _curpos(curpos), _pos(ipos){};

    reference operator*() {return *_curpos;}

    pointer Get() {return _curpos;}

    int GetPos() {return _pos;};

    VertexIter& operator++()
    {
      _curpos=_curpos+1;
      _pos++;
      return *this;
    }

    VertexIter operator++(int)
    {
      VertexIter old_it = *this;
      ++(*this);
      return old_it;
    }

    bool operator !=(VertexIter rhs){return _curpos!=rhs._curpos;};

  protected:
    Vector3<T>* _curpos;
    int _pos;
  };

  //----------------------------------------------------------------------------


  /**
  * @brief An ElementIter iterates over elements of a grid
  *
  */    
  class ElementIter
  {
  public:
  	
	  typedef Hexa  value_type;
	  typedef Hexa* pointer;
	  typedef Hexa& reference;
	  ElementIter(Hexa* curpos = NULL,int ipos =0) : _curpos(curpos),_ipos(ipos) {};

	  reference operator*() {return *_curpos;}

	  ElementIter& operator++()
	  {
		  _curpos=_curpos+1;
		  _ipos++;
		  return *this;
	  }

	  pointer Get(){return _curpos;};

	  int GetInt(){return _ipos;};

	  ElementIter operator++(int)
	  {
		  ElementIter old_it = *this;
		  ++(*this);
		  return old_it;
	  }

	  bool operator !=(ElementIter rhs){return _curpos!=rhs._curpos;};



  protected:
	 Hexa* _curpos;
	 int    _ipos;
  };
	
	//----------------------------------------------------------------------------

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
	  ElemVertIter(int *curpos = NULL,UnstructuredGrid<T,VTraits> *grid=NULL) : _curpos(curpos),m_pGrid(grid) {};

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
  /**
  * @brief An EdgeIter iterates over the edges of a CUnstrucuredGrid
  *
  */        
  class EdgeIter
  {
  public:
  	
	  typedef HexaEdge  value_type;
	  typedef HexaEdge* pointer;
	  typedef HexaEdge& reference;
		//Contructor initializes the edgeiter with NULL
	  EdgeIter(HexaEdge* curpos = NULL) : _curpos(curpos) {};

	  reference operator*() {return *_curpos;}

	  pointer Get() {return _curpos;}

	  EdgeIter& operator++()
	  {
		  _curpos=_curpos+1;
		  return *this;
	  }

	  EdgeIter operator++(int)
	  {
		  EdgeIter old_it = *this;
		  ++(*this);
		  return old_it;
	  }

	  bool operator !=(EdgeIter rhs){return _curpos!=rhs._curpos;};

  protected:
	 HexaEdge* _curpos;
  };
	
	//----------------------------------------------------------------------------	
  //VertexVertexIterator
  /**
  * @brief An VertexVertexIter iterates over the vertices attached to a vertex
  *
  */          
  class VertexVertexIter
  {
  public:
  	
	  typedef int  value_type;
	  typedef int* pointer;
	  typedef int& reference;
	  VertexVertexIter(int curpos = 0,UnstructuredGrid<T,VTraits> *grid=NULL,int iVert=0) : _curpos(curpos),m_pGrid(grid),m_iVert(iVert) {};

	  reference operator*() {return  m_pGrid->m_VertexVertex[m_iVert].m_iVertInd[_curpos];};

	  VertexVertexIter& operator++()
	  {
		  _curpos=_curpos+1;
		  return *this;
	  }

	  VertexVertexIter operator++(int)
	  {
		  VertexVertexIter old_it = *this;
		  ++(*this);
		  return old_it;
	  }

	  bool operator !=(VertexVertexIter rhs){return _curpos!=rhs._curpos;};

  protected:
	 int _curpos;
	 UnstructuredGrid<T,VTraits> *m_pGrid;
	 int m_iVert;
  };
  
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

  ElementIter ElemBegin();
  ElementIter ElemEnd();

  typename UnstructuredGrid<T,VTraits>::VertexVertexIter VertexVertexBegin(typename UnstructuredGrid<T,VTraits>::VertexIter vIter);
  typename UnstructuredGrid<T,VTraits>::VertexVertexIter VertexVertexEnd(typename UnstructuredGrid<T,VTraits>::VertexIter vIter);

  typename UnstructuredGrid<T,VTraits>::VertexVertexIter VertexVertexBegin(int iVert);
  typename UnstructuredGrid<T,VTraits>::VertexVertexIter VertexVertexEnd(int iVert);
  
  
  //the function returns an EdgeIter that points to the first edge
  typename UnstructuredGrid<T,VTraits>::EdgeIter EdgeBegin();
  //the function returns an EdgeIter that points to end of the edge array
  typename UnstructuredGrid<T,VTraits>::EdgeIter EdgeEnd();

  //the function returns a VertexIter that points to the first vertex
  typename UnstructuredGrid<T,VTraits>::VertexIter VertexBegin();
  typename UnstructuredGrid<T,VTraits>::VertexIter VertexEnd();
  
  //ElemVertIter GetElemVertIter(VertexIter vIter);
  typename UnstructuredGrid<T,VTraits>::ElemVertIter begin(typename UnstructuredGrid<T,VTraits>::VertexIter vIter);
  typename UnstructuredGrid<T,VTraits>::ElemVertIter end(typename UnstructuredGrid<T,VTraits>::VertexIter vIter);
      
      
  friend class VertexVertexIter;
  friend class ElemVertIter;
  friend class VertexIter;
  friend class VertElemIter;
  friend class ElementIter;
  friend class EdgeIter;	
  


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

  struct l0
  {
    bool operator()(const Connector &elem1,const Connector &elem2 ) 
    {
      return elem1.idata[0] < elem2.idata[0];
    }
  };


  struct l1
  {
    bool operator()(const  Connector &elem1,const  Connector &elem2 )
    {
      return elem1.idata[1] < elem2.idata[1];
    }
  };

  struct l2
  {
    bool operator()(const  Connector &elem1,const  Connector &elem2 )
    {
      return elem1.idata[2] < elem2.idata[2];
    }
  };


  struct l3
  {
    bool operator()(const  Connector &elem1,const  Connector &elem2 )
    {
      return elem1.idata[3] < elem2.idata[3];
    }
  };

  int m_iSize;
  Connector *pList;

  void sort_list()
  {
    std::stable_sort(pList,pList+m_iSize,l3());
    std::stable_sort(pList,pList+m_iSize,l2());
    std::stable_sort(pList,pList+m_iSize,l1());
    std::stable_sort(pList,pList+m_iSize,l0());
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


template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::ElementIter UnstructuredGrid<T,Traits>::ElemBegin()
{  
  return ElementIter(hexas_,0);
};//end 

template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::ElementIter UnstructuredGrid<T,Traits>::ElemEnd()
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
inline typename UnstructuredGrid<T,Traits>::VertexVertexIter UnstructuredGrid<T,Traits>::VertexVertexBegin(typename UnstructuredGrid<T,Traits>::VertexIter vIter)
{
  int diff = (vIter.Get()-vertexCoords_);
  return VertexVertexIter(0,this,diff);
};//end 

template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::VertexVertexIter UnstructuredGrid<T,Traits>::VertexVertexEnd(typename UnstructuredGrid<T,Traits>::VertexIter vIter)
{
  int diff = (vIter.Get()-vertexCoords_);
  return VertexVertexIter(6,this,diff);
};//end 

template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::VertexVertexIter UnstructuredGrid<T,Traits>::VertexVertexBegin(int iVert)
{
  return VertexVertexIter(0,this,iVert);
};//end 

template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::VertexVertexIter UnstructuredGrid<T,Traits>::VertexVertexEnd(int iVert)
{
  int iEnd = m_VertexVertex[iVert].m_iNeighbors;
  return VertexVertexIter(iEnd,this,iVert);
};//end 

template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::ElemVertIter UnstructuredGrid<T,Traits>::begin(typename UnstructuredGrid<T,Traits>::VertexIter vIter)
{
  int diff = (vIter.Get()-vertexCoords_);
  return ElemVertIter(&elementsAtVertex_[elementsAtVertexIdx_[diff]],this);
};//end 

template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::ElemVertIter UnstructuredGrid<T,Traits>::end(typename UnstructuredGrid<T,Traits>::VertexIter vIter)
{
  int diff = (vIter.Get()-vertexCoords_);
  return ElemVertIter(&elementsAtVertex_[elementsAtVertexIdx_[diff+1]],this);
};//end 

template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::EdgeIter UnstructuredGrid<T,Traits>::EdgeBegin()
{
  return EdgeIter(verticesAtEdge_);
};//end 

template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::EdgeIter UnstructuredGrid<T,Traits>::EdgeEnd()
{
  return EdgeIter(verticesAtEdge_ + (nmt_));
};//end 
	
template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::VertexIter UnstructuredGrid<T,Traits>::VertexBegin()
{
  return VertexIter(vertexCoords_,0);
};//end 

template<class T,class Traits>
inline typename UnstructuredGrid<T,Traits>::VertexIter UnstructuredGrid<T,Traits>::VertexEnd()
{
  return VertexIter(vertexCoords_ + (nvt_),nvt_);
};//end

typedef UnstructuredGrid<Real, DTraits> CUnstrGrid;
typedef UnstructuredGrid<Real, DTraits> CUnstrGridr;
typedef UnstructuredGrid<float, FTraits> UnstrGridf;

}

#endif
