
#ifdef WIN32
#pragma once
#endif

#ifndef _UNSTRUCTUREDGRID_H
#define _UNSTRUCTUREDGRID_H

//===================================================
//					DEFINITIONS
//===================================================

//===================================================
//					INCLUDES
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
//					Class
//===================================================

namespace i3d {

///@cond HIDDEN_SYMBOLS
typedef struct
{
  CVector3<Real> vVec;
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
	CVector3<Real> vRef;
  CVector3<Real> vNormal;  
};


class DistQueueEntry
{
public:
	
	CVector3<Real> *vPoint;
	DTraits *pTraits;
	int     iVert;

};

/**
* @brief Class that represents a hexahedron
*
*/
class CHexa
{
public:
  CHexa()
  {
	memset(m_iAdj,-1,6*sizeof(int));
	memset(m_iFaces,-1,6*sizeof(int));
	memset(m_iVertInd,-1,8*sizeof(int));
	memset(m_iEdges,-1,12*sizeof(int));
  };

  CHexa& operator=(const CHexa &hex)
  {
	memcpy(m_iVertInd,hex.m_iVertInd,8*sizeof(int));
	memcpy(m_iEdges,hex.m_iEdges,12*sizeof(int));
	memcpy(m_iFaces,hex.m_iFaces,6*sizeof(int));
	memcpy(m_iAdj,hex.m_iAdj,6*sizeof(int));
	return *this;
  };

  CHexa(const CHexa &hex)
  {
	memcpy(m_iVertInd,hex.m_iVertInd,8*sizeof(int));
	memcpy(m_iEdges,hex.m_iEdges,12*sizeof(int));
	memcpy(m_iFaces,hex.m_iFaces,6*sizeof(int));
	memcpy(m_iAdj,hex.m_iAdj,6*sizeof(int));
  };

  /**
   * The array maps the indices of the 8 local nodes to the
   * the global vertex array, so that the vertex coordinates
   * can be accessed.
   **/
  int m_iVertInd[8];
  /**
   * The array maps the indices of the vertices of the local 12 edges 
   * to the global vertex array, so that the coordinates of the edge
   * vertices can be accessed
   */
  int m_iEdges[12];
  /**
   * The array maps the local vertex indices of the 6 faces to
   * the global vertex array, so that the coordinates of the vertices
   * can be accessed
   */
  int m_iFaces[6];
  /**
   * The array stores the element id of the neighbouring hexas
   * at the 6 faces of the hexa. If there is no neighbour at a
   * face the array stores a -1 at this position.
   */
  int m_iAdj[6];
  
};

/**
* @brief Class that represents a quadrilateral face in 3d
*
*/
class CFace
{
public:
  int m_iVertInd[4];
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
class CEdge
{
public:
  int m_iVertInd[2];
};

/**
* @brief An unstructured mesh class in 3d
*
* An unstructured mesh class in 3d that consists of vertices, edges, faces and hexahedrons.
*  
*
*/
template<class T,class VTraits = DTraits>
class CUnstructuredGrid
{
public:
	CUnstructuredGrid(void);
	~CUnstructuredGrid(void);
  
  CVector3<T> *m_pVertexCoords;

  // Vertices Adjacent to an Element.
  // Handle to h_IverticesAtElement=array [1..NVE,1..NEL] of integer
  // For each element the node numbers of the corner-vertices
  // in mathematically positive sense.
  // On pure triangular meshes, there is NVE=3. On mixed or pure quad
  // meshes, there is NVE=4. In this case, there is 
  // IverticesAtElement(4,.)=0 for a triangle in a quad mesh.
  // This is a handle to the old KVERT array.
  CHexa *m_pHexas;

  // Handle to 
  //       p_IverticesAtBoundary = array [1..NVBD] of integer.
  // This array contains a list of all vertices on the (real) boundary
  //in mathematically positive sense.
  // The boundary vertices of boundary component i are saved at
  //        p_IboundaryCpIdx(i)..p_IboundaryCpIdx(i+1)-1.
  // This is the old KVBD array.
  int *m_piVertAtBdr;

  //      p_IfacesAtBoundary = array [1..NMBD] of integer.
  //This array contains a list of all edges on the (real) boundary
  //with increasing number.
  //The boundary edges of boundary component i are saved at
  //       p_IboundaryCpFacesIdx(i)..p_IboundaryCpFacesIdx(i+1)-1.
  int *m_iFacesAtBdr;

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
  int *m_iElAtBdr;

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
  CFace *m_VertAtFace;

  // Vertices Adjacent to an Edge. 
  // Handle to 
  //       p_IverticesAtEdge = array [1..2,1..NMT]
  // The numbers of the two vertices adjacent to an edge IMT. 
  CEdge *m_VertAtEdge;

  // p_IneighboursAtElement = array [1..TRIA_MAXNME2D,1..NEL] of integer
  // For each element, the numbers of adjacent elements
  // in mathematically positive sense, meeting the element in an edge.
  // p_RneighbourElement(IEL)\%Ineighbours(.) describes the elements adjacent 
  // to IEL along the edges (p_RedgesOnElement(IEL)\%Iedges(.)-NVT).
  // This is the old KADJ array.
  CHexa *m_NeighAtEl;
	
	CVertexVertex *m_VertexVertex;

  VTraits *m_myTraits;

  int *m_piElAtVertIdx;
  
  int *m_piElAtVert;
  
  int *m_piVertexOrder;

  // the total number of vertices
  int m_iNVT;

  // the total number of edges
  int m_iNMT;

  // the total number of faces
  int m_iNAT;

  // the total number of elements
  int m_iNEL;
  
  int m_iRefinementLevel;
  
  CVector3<T> m_vMin;
  CVector3<T> m_vMax;  
  
  CVector3<T>& Vertex(int i) {return m_pVertexCoords[i];};
  const CVector3<T>& Vertex(int i) const {return m_pVertexCoords[i];};

  VTraits& VertexTrait(int i) {return m_myTraits[i];};
  const VTraits& VertexTrait(int i) const {return m_myTraits[i];};
	

  void VertAtBdr();
  
  void InitUnitCube();
  
  void InitMeshFromFile(const char *strFileName);
  
  void InitStdMesh();

  void VertexOrderXYZ();

  CAABB3<T> GetAABB()
  {
    return CAABB3<T>(m_vMin,m_vMax);
  };
  
  /**
   * Applies a regular refinement strategy on the mesh
   */
  void Refine();
  
  /**
   * Initializes a grid for a box, specified by 2 points in space
   * @param xmin Minimal x-coordinate
   * @param ymin Minimal y-coordinate
   * @param zmin Minimal z-coordinate
   * @param xmax Maximal x-coordinate
   * @param ymax Maximal y-coordinate
   * @param zmaz Maximal z-coordinate
   */
  void InitCube(T xmin, T ymin, T zmin, T xmax, T ymax, T zmax);
  
  /**
   * Initializes a grid from an aabb
   * @param aabb an axis-aligned bounding box
   */
  void InitCubeFromAABB(const CAABB3<T> &aabb);  

	//----------------------------------------------------------------------------
	//Class VertexIter: the vertex iterator iterates over all vertices of the mesh
  /**
  * @brief A VertexIter iterates over vertices of a CUnstructuredGrid
  *
  */  
  class VertexIter
  {
  public:
  	
	  typedef CVector3<T>  value_type;
	  typedef CVector3<T>* pointer;
	  typedef CVector3<T>& reference;
	  VertexIter(CVector3<T>* curpos = NULL, int ipos=0) : _curpos(curpos), _pos(ipos){};

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
	 CVector3<T>* _curpos;
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
  	
	  typedef CHexa  value_type;
	  typedef CHexa* pointer;
	  typedef CHexa& reference;
	  ElementIter(CHexa* curpos = NULL,int ipos =0) : _curpos(curpos),_ipos(ipos) {};

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
	 CHexa* _curpos;
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
  	
	  typedef CHexa  value_type;
	  typedef CHexa* pointer;
	  typedef CHexa& reference;
	  ElemVertIter(int *curpos = NULL,CUnstructuredGrid<T,VTraits> *grid=NULL) : _curpos(curpos),m_pGrid(grid) {};

	  reference operator*() {return m_pGrid->m_pHexas[*_curpos];}

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
	 CUnstructuredGrid<T,VTraits> *m_pGrid;
  };
	
	//----------------------------------------------------------------------------
  /**
  * @brief An EdgeIter iterates over the edges of a CUnstrucuredGrid
  *
  */        
  class EdgeIter
  {
  public:
  	
	  typedef CEdge  value_type;
	  typedef CEdge* pointer;
	  typedef CEdge& reference;
		//Contructor initializes the edgeiter with NULL
	  EdgeIter(CEdge* curpos = NULL) : _curpos(curpos) {};

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
	 CEdge* _curpos;
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
	  VertexVertexIter(int curpos = 0,CUnstructuredGrid<T,VTraits> *grid=NULL,int iVert=0) : _curpos(curpos),m_pGrid(grid),m_iVert(iVert) {};

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
	 int m_iVert;
	 CUnstructuredGrid<T,VTraits> *m_pGrid;
  };
  
	//----------------------------------------------------------------------------	

  /**
  * @brief An VertElemIter iterates over the vertices of a CHexa
  *
  */    
  class VertElemIter
  {
  public:
  	
	  typedef CVector3<T>  value_type;
	  typedef CVector3<T>* pointer;
	  typedef CVector3<T>& reference;
	  VertElemIter(int curpos = 0,CUnstructuredGrid<T,VTraits> *grid=NULL,CHexa *pHexa=NULL,int iHexa=0) : _curpos(curpos),m_pGrid(grid),m_pHexa(pHexa),m_iHexa(iHexa) {};

	  reference operator*() {return  m_pGrid->m_pVertexCoords[m_pGrid->m_pHexas[m_iHexa].m_iVertInd[_curpos]];};

	  VertElemIter& operator++()
	  {
		  _curpos=_curpos+1;
		  return *this;
	  }

	  int GetInt(){return m_pGrid->m_pHexas[m_iHexa].m_iVertInd[_curpos];};

	  VertElemIter operator++(int)
	  {
		  VertElemIter old_it = *this;
		  ++(*this);
		  return old_it;
	  }

	  bool operator !=(VertElemIter rhs){return _curpos!=rhs._curpos;};

  protected:
	 int _curpos;
	 CHexa *m_pHexa;
	 int m_iHexa;
	 CUnstructuredGrid<T,VTraits> *m_pGrid;
  };
	//----------------------------------------------------------------------------	
	
  typename CUnstructuredGrid<T,VTraits>::VertElemIter VertElemBegin(CHexa* pHexa);
  typename CUnstructuredGrid<T,VTraits>::VertElemIter VertElemEnd(CHexa* pHexa);

  ElementIter ElemBegin();
  ElementIter ElemEnd();

	typename CUnstructuredGrid<T,VTraits>::VertexVertexIter VertexVertexBegin(typename CUnstructuredGrid<T,VTraits>::VertexIter vIter);
	typename CUnstructuredGrid<T,VTraits>::VertexVertexIter VertexVertexEnd(typename CUnstructuredGrid<T,VTraits>::VertexIter vIter);

	typename CUnstructuredGrid<T,VTraits>::VertexVertexIter VertexVertexBegin(int iVert);
	typename CUnstructuredGrid<T,VTraits>::VertexVertexIter VertexVertexEnd(int iVert);
	
	
	//the function returns an EdgeIter that points to the first edge
	typename CUnstructuredGrid<T,VTraits>::EdgeIter EdgeBegin();
	//the function returns an EdgeIter that points to end of the edge array
	typename CUnstructuredGrid<T,VTraits>::EdgeIter EdgeEnd();

	//the function returns a VertexIter that points to the first vertex
  typename CUnstructuredGrid<T,VTraits>::VertexIter VertexBegin();
  typename CUnstructuredGrid<T,VTraits>::VertexIter VertexEnd();
  
  //ElemVertIter GetElemVertIter(VertexIter vIter);
  typename CUnstructuredGrid<T,VTraits>::ElemVertIter begin(typename CUnstructuredGrid<T,VTraits>::VertexIter vIter);
  typename CUnstructuredGrid<T,VTraits>::ElemVertIter end(typename CUnstructuredGrid<T,VTraits>::VertexIter vIter);
	
	
  friend class CUnstructuredGrid<T,VTraits>::VertexVertexIter;
  friend class CUnstructuredGrid<T,VTraits>::ElemVertIter;
  friend class CUnstructuredGrid<T,VTraits>::VertexIter;
  friend class CUnstructuredGrid<T,VTraits>::VertElemIter;
  friend class CUnstructuredGrid<T,VTraits>::ElementIter;
  friend class CUnstructuredGrid<T,VTraits>::EdgeIter;	
  


private:
  void RefineRaw();
  void GenEdgesAtEl();
  void GenElAtVert();
  void GenNeighboursAtEl();
  void GenFacesAtEl();
  void GenVertAtEdg();
  void GenVertAtFac();
	void GenVertexVertex();
  void CleanExtended();
  int FindSmlstEl(int ivt1, int ivt2, int iel)
  {
	int iSmlstEl=iel;
	int iStart1,iStart2;
	int iEnd1,iEnd2;
	int iel1,iel2;
	iStart1=m_piElAtVertIdx[ivt1];
	iStart2=m_piElAtVertIdx[ivt2];
	iEnd1=m_piElAtVertIdx[ivt1+1]-1;
	iEnd2=m_piElAtVertIdx[ivt2+1]-1;

	for(int i=iStart1;i<iEnd1;i++)
	{
	  iel1  = m_piElAtVert[i];

	  for(int j=iStart2;j<iEnd2;j++)
	  {
		iel2 = m_piElAtVert[j];

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
inline typename CUnstructuredGrid<T,Traits>::ElementIter CUnstructuredGrid<T,Traits>::ElemBegin()
{
	
	return ElementIter(m_pHexas,0);
};//end 

template<class T,class Traits>
inline typename CUnstructuredGrid<T,Traits>::ElementIter CUnstructuredGrid<T,Traits>::ElemEnd()
{
  return ElementIter(m_pHexas+m_iNEL,m_iNEL);
};//end 

template<class T,class Traits>
inline typename CUnstructuredGrid<T,Traits>::VertElemIter CUnstructuredGrid<T,Traits>::VertElemBegin(CHexa* pHexa)
{
	int diff = (pHexa-m_pHexas);
	return VertElemIter(0,this,pHexa,diff);
};//end 

template<class T,class Traits>
inline typename CUnstructuredGrid<T,Traits>::VertElemIter CUnstructuredGrid<T,Traits>::VertElemEnd(CHexa* pHexa)
{
	int diff = (pHexa-m_pHexas);
	return VertElemIter(8,this,pHexa,diff);
};//end

template<class T,class Traits>
inline typename CUnstructuredGrid<T,Traits>::VertexVertexIter CUnstructuredGrid<T,Traits>::VertexVertexBegin(typename CUnstructuredGrid<T,Traits>::VertexIter vIter)
{
	int diff = (vIter.Get()-m_pVertexCoords);
	return VertexVertexIter(0,this,diff);
};//end 

template<class T,class Traits>
inline typename CUnstructuredGrid<T,Traits>::VertexVertexIter CUnstructuredGrid<T,Traits>::VertexVertexEnd(typename CUnstructuredGrid<T,Traits>::VertexIter vIter)
{
	int diff = (vIter.Get()-m_pVertexCoords);
	return VertexVertexIter(6,this,diff);
};//end 

template<class T,class Traits>
inline typename CUnstructuredGrid<T,Traits>::VertexVertexIter CUnstructuredGrid<T,Traits>::VertexVertexBegin(int iVert)
{
	return VertexVertexIter(0,this,iVert);
};//end 

template<class T,class Traits>
inline typename CUnstructuredGrid<T,Traits>::VertexVertexIter CUnstructuredGrid<T,Traits>::VertexVertexEnd(int iVert)
{
	int iEnd = m_VertexVertex[iVert].m_iNeighbors;
	return VertexVertexIter(iEnd,this,iVert);
};//end 

template<class T,class Traits>
inline typename CUnstructuredGrid<T,Traits>::ElemVertIter CUnstructuredGrid<T,Traits>::begin(typename CUnstructuredGrid<T,Traits>::VertexIter vIter)
{
	int diff = (vIter.Get()-m_pVertexCoords);
	return ElemVertIter(&m_piElAtVert[m_piElAtVertIdx[diff]],this);
};//end 

template<class T,class Traits>
inline typename CUnstructuredGrid<T,Traits>::ElemVertIter CUnstructuredGrid<T,Traits>::end(typename CUnstructuredGrid<T,Traits>::VertexIter vIter)
{
	int diff = (vIter.Get()-m_pVertexCoords);
	return ElemVertIter(&m_piElAtVert[m_piElAtVertIdx[diff+1]],this);
};//end 

template<class T,class Traits>
inline typename CUnstructuredGrid<T,Traits>::EdgeIter CUnstructuredGrid<T,Traits>::EdgeBegin()
{
	return EdgeIter(m_VertAtEdge);
};//end 

template<class T,class Traits>
inline typename CUnstructuredGrid<T,Traits>::EdgeIter CUnstructuredGrid<T,Traits>::EdgeEnd()
{
	return EdgeIter(m_VertAtEdge + (m_iNMT));
};//end 
	
template<class T,class Traits>
inline typename CUnstructuredGrid<T,Traits>::VertexIter CUnstructuredGrid<T,Traits>::VertexBegin()
{
	return VertexIter(m_pVertexCoords,0);
};//end 

template<class T,class Traits>
inline typename CUnstructuredGrid<T,Traits>::VertexIter CUnstructuredGrid<T,Traits>::VertexEnd()
{
	return VertexIter(m_pVertexCoords + (m_iNVT),m_iNVT);
};//end

typedef CUnstructuredGrid<double, DTraits> CUnstrGrid;
typedef CUnstructuredGrid<Real, DTraits> CUnstrGridr;

}

#endif
