#include "unstructuredgrid.h"

namespace i3d {

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
CUnstructuredGrid<T,Traits>::CUnstructuredGrid(void)
{
  m_pVertexCoords = NULL;
  m_pHexas        = NULL;
  m_piVertAtBdr   = NULL;
  m_iFacesAtBdr   = NULL;
  m_iElAtBdr      = NULL;
  m_VertAtFace    = NULL;
  m_VertAtEdge    = NULL;
  m_NeighAtEl     = NULL;
  m_VertexVertex  = NULL;
  m_myTraits      = NULL;
  m_piElAtVertIdx = NULL;
  m_piElAtVert    = NULL;
  m_piVertexOrder = NULL;
  m_iRefinementLevel = 0;
};

template<class T,class Traits>
CUnstructuredGrid<T,Traits>::~CUnstructuredGrid(void)
{
  
  if(m_pVertexCoords != NULL)
  {
    delete[] m_pVertexCoords;    
    m_pVertexCoords = NULL;
  }
  
  if(m_piVertexOrder != NULL)
  {
    delete[] m_piVertexOrder;    
    m_piVertexOrder = NULL;
  }  
  
  if(m_pHexas != NULL)
  {
    delete[] m_pHexas;
    m_pHexas = NULL;
  }
  
  if(m_piVertAtBdr != NULL)
  {
    delete[] m_piVertAtBdr;    
    m_piVertAtBdr = NULL;
  }  
  
  if(m_iFacesAtBdr != NULL)
  {
    delete[] m_iFacesAtBdr;
    m_iFacesAtBdr = NULL;
  }
  
  if(m_iElAtBdr != NULL)
  {
    delete[] m_iElAtBdr;
    m_iElAtBdr = NULL;
  }  
  
  if(m_VertAtFace != NULL)
  {
    delete[] m_VertAtFace;
    m_VertAtFace = NULL;
  }  

  if(m_VertAtEdge != NULL)
  {
    delete[] m_VertAtEdge;
    m_VertAtEdge = NULL;
  }  

  if(m_NeighAtEl != NULL)
  {
    delete[] m_NeighAtEl;
    m_NeighAtEl = NULL;
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

  if(m_piElAtVertIdx != NULL)
  {
    delete[] m_piElAtVertIdx;
    m_piElAtVertIdx = NULL;
  }
  
  if(m_piElAtVert != NULL)
  {
    delete[] m_piElAtVert;
    m_piElAtVert = NULL;
  }    
  
};

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::InitUnitCube()
{
  // the total number of vertices
  m_iNVT = 8;

  // the total number of elements
  m_iNEL = 1;

  this->m_pVertexCoords = new CVector3<T>[m_iNVT+1];
  m_piVertAtBdr         = new int[m_iNVT];  
  m_pHexas              = new CHexa[1];

  m_myTraits            = new Traits[m_iNVT];
  for(int i=0;i<m_iNVT;i++)
    memset(&m_myTraits[i],0,sizeof(Traits));

  m_pVertexCoords[0] = CVector3<T>(-1.0,-1.0,-1.0);
  m_pVertexCoords[1] = CVector3<T>(1.0,-1.0,-1.0);
  m_pVertexCoords[2] = CVector3<T>(1.0,1.0,-1.0);
  m_pVertexCoords[3] = CVector3<T>(-1.0,1.0,-1.0);
  m_pVertexCoords[4] = CVector3<T>(-1.0,-1.0,1.0);
  m_pVertexCoords[5] = CVector3<T>(1.0,-1.0,1.0);
  m_pVertexCoords[6] = CVector3<T>(1.0,1.0,1.0);
  m_pVertexCoords[7] = CVector3<T>(-1.0,1.0,1.0);
  m_piVertAtBdr[0]   = 0;
  m_piVertAtBdr[1]   = 1;
  m_piVertAtBdr[2]   = 2;
  m_piVertAtBdr[3]   = 3;
  m_piVertAtBdr[4]   = 4;
  m_piVertAtBdr[5]   = 5;
  m_piVertAtBdr[6]   = 6;
  m_piVertAtBdr[7]   = 7;

  m_pHexas[0].m_iVertInd[0]      = 0;
  m_pHexas[0].m_iVertInd[1]      = 1;
  m_pHexas[0].m_iVertInd[2]      = 2;
  m_pHexas[0].m_iVertInd[3]      = 3;
  m_pHexas[0].m_iVertInd[4]      = 4;
  m_pHexas[0].m_iVertInd[5]      = 5;
  m_pHexas[0].m_iVertInd[6]      = 6;
  m_pHexas[0].m_iVertInd[7]      = 7;

};

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::InitMeshFromFile(const char *strFileName)
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
	in >> m_iNEL;
	in >> m_iNVT;
	in.getline(strLine,256);
	in.getline(strLine,256);

  this->m_pVertexCoords = new CVector3<T>[m_iNVT+1];
  m_pHexas              = new CHexa[m_iNEL];
  m_myTraits            = new Traits[m_iNVT];
	memset(m_myTraits,0,m_iNVT*sizeof(Traits));
  
//   m_vMax=CVector3<T>(xmax,ymax,zmax);
//   m_vMin=CVector3<T>(xmin,ymin,zmin);
	for(int i=0;i<m_iNVT;i++)
	{
		CVector3<T> vec;
		in >> vec.x;
		in >> vec.y;
		in >> vec.z;
		m_pVertexCoords[i] = vec;
		in.getline(strLine,256);
	}

  m_vMax=CVector3<T>(m_pVertexCoords[0].x,m_pVertexCoords[0].y,m_pVertexCoords[0].z);
  m_vMin=CVector3<T>(m_pVertexCoords[0].x,m_pVertexCoords[0].y,m_pVertexCoords[0].z);

	for(int i=1;i<m_iNVT;i++)
	{
    if(m_vMin.x > m_pVertexCoords[i].x)
      m_vMin.x = m_pVertexCoords[i].x;

    if(m_vMin.y > m_pVertexCoords[i].y)
      m_vMin.y = m_pVertexCoords[i].y;

    if(m_vMin.z > m_pVertexCoords[i].z)
      m_vMin.z = m_pVertexCoords[i].z;

    if(m_vMax.x < m_pVertexCoords[i].x)
      m_vMax.x = m_pVertexCoords[i].x;

    if(m_vMax.y < m_pVertexCoords[i].y)
      m_vMax.y = m_pVertexCoords[i].y;

    if(m_vMax.z < m_pVertexCoords[i].z)
      m_vMax.z = m_pVertexCoords[i].z;
	}

	in.getline(strLine,256);
	for(int i=0;i<m_iNEL;i++)
	{
		for(int j=0;j<8;j++)
		{
			in >> m_pHexas[i].m_iVertInd[j];
			m_pHexas[i].m_iVertInd[j]--;
		}
		in.getline(strLine,256);
	}
	in.getline(strLine,256);
	
	std::vector<int> nodalProperties;
	for(int i=0;i<m_iNVT;i++)
	{
		int iprop;
		in >> iprop;
		nodalProperties.push_back(iprop);
		in.getline(strLine,256);
	}
	
  m_piVertAtBdr         = new int[nodalProperties.size()];  
	for(int i=0;i<nodalProperties.size();i++)
	{
		m_piVertAtBdr[i]=nodalProperties[i];
	}

	in.close();
	cout<<"Grid: "<<strFileName<<" loaded successfully."<<endl;
}

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::InitCube(T xmin, T ymin, T zmin, T xmax, T ymax, T zmax)
{
  // the total number of vertices
  m_iNVT = 8;

  // the total number of elements
  m_iNEL = 1;

  this->m_pVertexCoords = new CVector3<T>[m_iNVT+1];
  m_piVertAtBdr         = new int[m_iNVT];  
  m_pHexas              = new CHexa[1];

  m_myTraits            = new Traits[m_iNVT];
  
  m_vMax=CVector3<T>(xmax,ymax,zmax);
  m_vMin=CVector3<T>(xmin,ymin,zmin);

  m_pVertexCoords[0] = CVector3<T>(xmin,ymin,zmin);
  m_pVertexCoords[1] = CVector3<T>(xmax,ymin,zmin);
  m_pVertexCoords[2] = CVector3<T>(xmax,ymax,zmin);
  m_pVertexCoords[3] = CVector3<T>(xmin,ymax,zmin);
  m_pVertexCoords[4] = CVector3<T>(xmin,ymin,zmax);
  m_pVertexCoords[5] = CVector3<T>(xmax,ymin,zmax);
  m_pVertexCoords[6] = CVector3<T>(xmax,ymax,zmax);
  m_pVertexCoords[7] = CVector3<T>(xmin,ymax,zmax);
  m_piVertAtBdr[0]   = 0;
  m_piVertAtBdr[1]   = 1;
  m_piVertAtBdr[2]   = 2;
  m_piVertAtBdr[3]   = 3;
  m_piVertAtBdr[4]   = 4;
  m_piVertAtBdr[5]   = 5;
  m_piVertAtBdr[6]   = 6;
  m_piVertAtBdr[7]   = 7;

  m_pHexas[0].m_iVertInd[0]      = 0;
  m_pHexas[0].m_iVertInd[1]      = 1;
  m_pHexas[0].m_iVertInd[2]      = 2;
  m_pHexas[0].m_iVertInd[3]      = 3;
  m_pHexas[0].m_iVertInd[4]      = 4;
  m_pHexas[0].m_iVertInd[5]      = 5;
  m_pHexas[0].m_iVertInd[6]      = 6;
  m_pHexas[0].m_iVertInd[7]      = 7;

};

template<class T, class Traits>
void CUnstructuredGrid<T,Traits>::InitCubeFromAABB(const CAABB3<T> &aabb)
{
  // the total number of vertices
  m_iNVT = 8;

  // the total number of elements
  m_iNEL = 1;

  this->m_pVertexCoords = new CVector3<T>[m_iNVT+1];
  m_piVertAtBdr         = new int[m_iNVT];  
  m_pHexas              = new CHexa[1];

  m_myTraits            = new Traits[m_iNVT];
  
  m_vMax=aabb.m_Verts[1];
  m_vMin=aabb.m_Verts[0];

  m_pVertexCoords[0] = m_vMin;                                             
  m_pVertexCoords[1] = CVector3<T>(m_vMax.x,m_vMin.y,m_vMin.z);            
  m_pVertexCoords[2] = CVector3<T>(m_vMax.x,m_vMax.y,m_vMin.z);            
  m_pVertexCoords[3] = CVector3<T>(m_vMin.x,m_vMax.y,m_vMin.z);            
  m_pVertexCoords[4] = CVector3<T>(m_vMin.x,m_vMin.y,m_vMax.z);            
  m_pVertexCoords[5] = CVector3<T>(m_vMax.x,m_vMin.y,m_vMax.z);            
  m_pVertexCoords[6] = CVector3<T>(m_vMax.x,m_vMax.y,m_vMax.z);            
  m_pVertexCoords[7] = CVector3<T>(m_vMin.x,m_vMax.y,m_vMax.z);            
  m_piVertAtBdr[0]   = 0;
  m_piVertAtBdr[1]   = 1;
  m_piVertAtBdr[2]   = 2;
  m_piVertAtBdr[3]   = 3;
  m_piVertAtBdr[4]   = 4;
  m_piVertAtBdr[5]   = 5;
  m_piVertAtBdr[6]   = 6;
  m_piVertAtBdr[7]   = 7;

  m_pHexas[0].m_iVertInd[0]      = 0;
  m_pHexas[0].m_iVertInd[1]      = 1;
  m_pHexas[0].m_iVertInd[2]      = 2;
  m_pHexas[0].m_iVertInd[3]      = 3;
  m_pHexas[0].m_iVertInd[4]      = 4;
  m_pHexas[0].m_iVertInd[5]      = 5;
  m_pHexas[0].m_iVertInd[6]      = 6;
  m_pHexas[0].m_iVertInd[7]      = 7;
  
}

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::VertexOrderXYZ()
{

  std::vector<sPerm> permArray;
  
  if(m_piVertexOrder == NULL)
    m_piVertexOrder = new int[m_iNVT];

  for(int i=0;i<m_iNVT;i++)
  {
    sPerm perm;
    perm.vVec = m_pVertexCoords[i];
    perm.index  = i;
    permArray.push_back(perm);
  }
  
  std::stable_sort(permArray.begin(),permArray.end(),coordz());
  std::stable_sort(permArray.begin(),permArray.end(),coordy());
  std::stable_sort(permArray.begin(),permArray.end(),coordx());
  
  for(int i=0;i<m_iNVT;i++)
  {
    m_piVertexOrder[i]=permArray[i].index;
  }  
  
};

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::GenElAtVert()
{

  if(m_piElAtVertIdx == NULL)
	m_piElAtVertIdx = new int[m_iNVT+1];

  memset(m_piElAtVertIdx,0,(m_iNVT+1)*sizeof(int));

  for(int i=0;i<m_iNEL;i++)
  {
	for(int j=0;j<8;j++)
	{
	  int vindex=m_pHexas[i].m_iVertInd[j]+1;
	  m_piElAtVertIdx[vindex]++;
	}//end for
  }//end for
  
  m_piElAtVertIdx[0]=0;
  for(int i=1;i<m_iNVT+1;i++)
  {
	m_piElAtVertIdx[i]+=m_piElAtVertIdx[i-1];
  }//end for

  int isize = m_piElAtVertIdx[m_iNVT];
  m_piElAtVert = new int[isize];

  int *iaux = new int[m_iNVT+1];
  memcpy(iaux,m_piElAtVertIdx,(m_iNVT+1)*sizeof(int));

  for(int i=0;i<m_iNEL;i++)
  {
	for(int j=0;j<8;j++)
	{
	  int vindex=m_pHexas[i].m_iVertInd[j];
	  m_piElAtVert[iaux[vindex]]=i;
	  iaux[vindex]++;
	}//end for
  }//end for

  delete[] iaux;

};

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::GenNeighboursAtEl()
{

  typename CUnstructuredGrid<T,Traits>::ConnectorList list(6*m_iNEL);

  int nfaces=0;
  for(int i=0;i<m_iNEL;i++)
  {
    //!=========================================================  
    //! first face
	for(int k=0;k<4;k++)
	  list.pList[nfaces].idata[k] = m_pHexas[i].m_iVertInd[k];

	list.pList[nfaces].idata[4]   = i;

	list.pList[nfaces].idata[5]   = 0;
    nfaces++;
    //!=========================================================  
    //! second face
    list.pList[nfaces].idata[0] = m_pHexas[i].m_iVertInd[0];
    list.pList[nfaces].idata[1] = m_pHexas[i].m_iVertInd[1];
    list.pList[nfaces].idata[2] = m_pHexas[i].m_iVertInd[4];
    list.pList[nfaces].idata[3] = m_pHexas[i].m_iVertInd[5];

	list.pList[nfaces].idata[4]   = i;

	list.pList[nfaces].idata[5]   = 1;
	nfaces++;
    //!=========================================================  
    //! third face
    list.pList[nfaces].idata[0] = m_pHexas[i].m_iVertInd[1];
    list.pList[nfaces].idata[1] = m_pHexas[i].m_iVertInd[2];
    list.pList[nfaces].idata[2] = m_pHexas[i].m_iVertInd[5];
    list.pList[nfaces].idata[3] = m_pHexas[i].m_iVertInd[6];

	list.pList[nfaces].idata[4]   = i;

	list.pList[nfaces].idata[5]   = 2;
	nfaces++;
    //!=========================================================  
    //! fourth face
    list.pList[nfaces].idata[0] = m_pHexas[i].m_iVertInd[3];
    list.pList[nfaces].idata[1] = m_pHexas[i].m_iVertInd[2];
    list.pList[nfaces].idata[2] = m_pHexas[i].m_iVertInd[6];
    list.pList[nfaces].idata[3] = m_pHexas[i].m_iVertInd[7];

	list.pList[nfaces].idata[4]   = i;

	list.pList[nfaces].idata[5]   = 3;
	nfaces++;
    //!=========================================================  
    //! fifth face
    list.pList[nfaces].idata[0] = m_pHexas[i].m_iVertInd[0];
    list.pList[nfaces].idata[1] = m_pHexas[i].m_iVertInd[3];
    list.pList[nfaces].idata[2] = m_pHexas[i].m_iVertInd[7];
    list.pList[nfaces].idata[3] = m_pHexas[i].m_iVertInd[4];

	list.pList[nfaces].idata[4]   = i;

	list.pList[nfaces].idata[5]   = 4;
	nfaces++;
    //!=========================================================  
    //! sixth face
	for(int k=4;k<8;k++)
	  list.pList[nfaces].idata[k-4] = m_pHexas[i].m_iVertInd[k];

	list.pList[nfaces].idata[4]   = i;

	list.pList[nfaces].idata[5]   = 5;
	nfaces++;
  }//end for

  //sort the connector list
  list.sortdata();
  list.sort_list();

  //assign the neighbours at elements
  //traverse the connector list
  for(int k=1;k<6*m_iNEL;k++)
  {
    int j=0;
    while(list.pList[k-1].idata[j]==list.pList[k].idata[j])
    {
      j++;
    }
    if(j==4)
    {
      m_pHexas[list.pList[k-1].idata[4]].m_iAdj[list.pList[k-1].idata[5]]=list.pList[k].idata[4];
      m_pHexas[list.pList[k].idata[4]].m_iAdj[list.pList[k].idata[5]]=list.pList[k-1].idata[4];
    }
  }//end for

}


template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::GenEdgesAtEl()
{
  //the algorithm is as follows
  CEdge edges[12];
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
  memcpy(edges[0].m_iVertInd,edge0,2*sizeof(int));
  memcpy(edges[1].m_iVertInd,edge1,2*sizeof(int));
  memcpy(edges[2].m_iVertInd,edge2,2*sizeof(int));
  memcpy(edges[3].m_iVertInd,edge3,2*sizeof(int));
  memcpy(edges[4].m_iVertInd,edge4,2*sizeof(int));
  memcpy(edges[5].m_iVertInd,edge5,2*sizeof(int));
  memcpy(edges[6].m_iVertInd,edge6,2*sizeof(int));
  memcpy(edges[7].m_iVertInd,edge7,2*sizeof(int));
  memcpy(edges[8].m_iVertInd,edge8,2*sizeof(int));
  memcpy(edges[9].m_iVertInd,edge9,2*sizeof(int));
  memcpy(edges[10].m_iVertInd,edge10,2*sizeof(int));
  memcpy(edges[11].m_iVertInd,edge11,2*sizeof(int));

  for(int i=0;i<m_iNEL;i++)
  {
	
	for(int ied=0;ied<12;ied++)
	{
	  int iloc1 = edges[ied].m_iVertInd[0];
	  int iloc2 = edges[ied].m_iVertInd[1];

	  int iglob1 = m_pHexas[i].m_iVertInd[iloc1];
	  int iglob2 = m_pHexas[i].m_iVertInd[iloc2];

	  //findsmallestedge
	  int ismallest = FindSmlstEl(iglob1,iglob2,i);

	  if(ismallest >= i)
	  {
		m_pHexas[i].m_iEdges[ied]=nedges;
		nedges++;
	  }//end if
	  else
	  {
		for(int k=0;k<12;k++)
		{
		  int ed1 = m_pHexas[ismallest].m_iVertInd[edges[k].m_iVertInd[0]];
		  int ed2 = m_pHexas[ismallest].m_iVertInd[edges[k].m_iVertInd[1]];
		  if(((ed1==iglob1) && (ed2==iglob2)) || ((ed1==iglob2)&&(ed2==iglob1)))
		  {
			int foundEdge = m_pHexas[ismallest].m_iEdges[k];
			m_pHexas[i].m_iEdges[ied]=foundEdge;
			break;
		  }
		}//end for k
		
	  }//end else

	}//end for ied

  }//end for i

  m_iNMT = nedges;

};

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::GenFacesAtEl()
{
	int iFace = 0;
	for(int i=0;i<m_iNEL;i++)
	{
	  for(int j=0;j<6;j++)
	  {
		if((m_pHexas[i].m_iAdj[j]==-1) 
		   || (m_pHexas[i].m_iAdj[j]>i))
		{
		  m_pHexas[i].m_iFaces[j]=iFace;
		  iFace++;
		}
		else
		{
		  int jel = m_pHexas[i].m_iAdj[j];
		  for(int k=0;k<6;k++)
		  {
			if(m_pHexas[jel].m_iAdj[k]==i)
			{
			  m_pHexas[i].m_iFaces[j]=m_pHexas[jel].m_iFaces[k];
			  break;
			}
		  }//end k
		}//end else
	  }//end for
	}//end for

	m_iNAT=iFace;

}//End GenFacesAtEl

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::GenVertAtEdg()
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

  m_VertAtEdge = new CEdge[this->m_iNMT];

  for(int i=0;i<m_iNEL;i++)
  {
	for(int j=0;j<12;j++)
	{
	  int index = m_pHexas[i].m_iEdges[j];
	  m_VertAtEdge[index].m_iVertInd[0]=
		m_pHexas[i].m_iVertInd[edgeindex[j][0]];
      m_VertAtEdge[index].m_iVertInd[1]= 
		m_pHexas[i].m_iVertInd[edgeindex[j][1]];
	}//end for
  }//end for

};

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::GenVertAtFac()
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
  m_VertAtFace = new CFace[this->m_iNAT];
  for(int i=0;i<m_iNEL;i++)
  {
	for(int j=0;j<6;j++)
	{
	  int faceindex=m_pHexas[i].m_iFaces[j];
	  for(int k=0;k<4;k++)
	  {
		m_VertAtFace[faceindex].m_iVertInd[k]=
		  m_pHexas[i].m_iVertInd[facesHex[j][k]];	  
	  }
	}//end for j
  }//end for
};

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::GenVertexVertex()
{
	m_VertexVertex = new CVertexVertex[m_iNVT];
  for(int i=0;i<this->m_iNMT;i++)
	{
		int ia = m_VertAtEdge[i].m_iVertInd[0];
		int ib = m_VertAtEdge[i].m_iVertInd[1];
		int j=0;
		m_VertexVertex[ia].m_iVertInd[m_VertexVertex[ia].m_iNeighbors]=ib;
		m_VertexVertex[ia].m_iNeighbors++;
		m_VertexVertex[ib].m_iVertInd[m_VertexVertex[ib].m_iNeighbors]=ia;
		m_VertexVertex[ib].m_iNeighbors++;
	}
};

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::Refine()
{
  RefineRaw();
  CleanExtended();
  m_iRefinementLevel++;
};

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::RefineRaw()
{
  int iOffset=0;
  int iNVTnew = m_iNVT+m_iNMT+m_iNAT+m_iNEL;

  CVector3<T> *pVertexCoordsNew = new CVector3<T>[iNVTnew+1];

  int iNELnew=8*m_iNEL;

  for(int i=0;i<m_iNVT;i++)
  {
	pVertexCoordsNew[i]=m_pVertexCoords[i];
  }//end for

  iOffset+=m_iNVT;

  for(int i=0;i<m_iNMT;i++)
  {
	int ivt1=this->m_VertAtEdge[i].m_iVertInd[0];
	int ivt2=this->m_VertAtEdge[i].m_iVertInd[1];

	CVector3<T> vMid = T(0.5) * (m_pVertexCoords[ivt1]+m_pVertexCoords[ivt2]);

	pVertexCoordsNew[iOffset+i]=vMid;

  }//end for

  iOffset+=m_iNMT;

  for(int i=0;i<m_iNAT;i++)
  {
	CVector3<T> vMid(0,0,0);
	for(int j=0;j<4;j++)
	{
	  vMid+=m_pVertexCoords[m_VertAtFace[i].m_iVertInd[j]];
	}//end for

	pVertexCoordsNew[iOffset+i]=vMid*(T)0.25;

  }//end for
  
  iOffset+=m_iNAT;

  for(int i=0;i<m_iNEL;i++)
  {

	CVector3<T> vMid(0,0,0);
	for(int j=0;j<8;j++)
	{
	  vMid+=m_pVertexCoords[this->m_pHexas[i].m_iVertInd[j]];
	}//end for j

	pVertexCoordsNew[iOffset+i]=vMid*(T)0.125;

  }//end for i

  iOffset=m_iNEL;

  CHexa *pHexaNew = new CHexa[8*m_iNEL];

  for(int i=0;i<m_iNEL;i++)
  {
	
	int el[7];
	int edg2vert[12];
	int fac2vert[6];
	int midEl=m_iNVT+m_iNMT+m_iNAT+i;
	for(int k=0;k<12;k++)edg2vert[k]=m_iNVT+m_pHexas[i].m_iEdges[k];
	for(int k=0;k<6;k++)fac2vert[k]=m_iNVT+m_iNMT+m_pHexas[i].m_iFaces[k];
	for(int k=0;k<7;k++)el[k]=iOffset+k;
	
	// the 1st hexahedron
	pHexaNew[i].m_iVertInd[0]= m_pHexas[i].m_iVertInd[0];
	pHexaNew[i].m_iVertInd[1]= edg2vert[0];
	pHexaNew[i].m_iVertInd[2]= fac2vert[0];
	pHexaNew[i].m_iVertInd[3]= edg2vert[3];

	pHexaNew[i].m_iVertInd[4]= edg2vert[4];
	pHexaNew[i].m_iVertInd[5]= fac2vert[1];
	pHexaNew[i].m_iVertInd[6]= midEl;
	pHexaNew[i].m_iVertInd[7]= fac2vert[4];

	// the 2nd new hexahedron
	pHexaNew[el[0]].m_iVertInd[0]=m_pHexas[i].m_iVertInd[1];
	pHexaNew[el[0]].m_iVertInd[1]=edg2vert[1];
	pHexaNew[el[0]].m_iVertInd[2]=fac2vert[0];
	pHexaNew[el[0]].m_iVertInd[3]=edg2vert[0];
						 
	pHexaNew[el[0]].m_iVertInd[4]=edg2vert[5];
	pHexaNew[el[0]].m_iVertInd[5]=fac2vert[2];
	pHexaNew[el[0]].m_iVertInd[6]=midEl;
	pHexaNew[el[0]].m_iVertInd[7]=fac2vert[1];

	// the 3rd new hexahedron
	pHexaNew[el[1]].m_iVertInd[0]=m_pHexas[i].m_iVertInd[2];
	pHexaNew[el[1]].m_iVertInd[1]=edg2vert[2];
	pHexaNew[el[1]].m_iVertInd[2]=fac2vert[0];
	pHexaNew[el[1]].m_iVertInd[3]=edg2vert[1];
						 
	pHexaNew[el[1]].m_iVertInd[4]=edg2vert[6];
	pHexaNew[el[1]].m_iVertInd[5]=fac2vert[3];
	pHexaNew[el[1]].m_iVertInd[6]=midEl;
	pHexaNew[el[1]].m_iVertInd[7]=fac2vert[2];

	// the 4th new hexahedron
	pHexaNew[el[2]].m_iVertInd[0]=m_pHexas[i].m_iVertInd[3];
	pHexaNew[el[2]].m_iVertInd[1]=edg2vert[3];
	pHexaNew[el[2]].m_iVertInd[2]=fac2vert[0];
	pHexaNew[el[2]].m_iVertInd[3]=edg2vert[2];
						 
	pHexaNew[el[2]].m_iVertInd[4]=edg2vert[7];
	pHexaNew[el[2]].m_iVertInd[5]=fac2vert[4];
	pHexaNew[el[2]].m_iVertInd[6]=midEl;
	pHexaNew[el[2]].m_iVertInd[7]=fac2vert[3];

	// the 5th new hexahedron
	pHexaNew[el[3]].m_iVertInd[0]=m_pHexas[i].m_iVertInd[4];
	pHexaNew[el[3]].m_iVertInd[1]=edg2vert[11];
	pHexaNew[el[3]].m_iVertInd[2]=fac2vert[5];
	pHexaNew[el[3]].m_iVertInd[3]=edg2vert[8];
						 
	pHexaNew[el[3]].m_iVertInd[4]=edg2vert[4];
	pHexaNew[el[3]].m_iVertInd[5]=fac2vert[4];
	pHexaNew[el[3]].m_iVertInd[6]=midEl;
	pHexaNew[el[3]].m_iVertInd[7]=fac2vert[1];

	// the 6th new hexahedron
	pHexaNew[el[4]].m_iVertInd[0]=m_pHexas[i].m_iVertInd[5];
	pHexaNew[el[4]].m_iVertInd[1]=edg2vert[8];
	pHexaNew[el[4]].m_iVertInd[2]=fac2vert[5];
	pHexaNew[el[4]].m_iVertInd[3]=edg2vert[9];
						 
	pHexaNew[el[4]].m_iVertInd[4]=edg2vert[5];
	pHexaNew[el[4]].m_iVertInd[5]=fac2vert[1];
	pHexaNew[el[4]].m_iVertInd[6]=midEl;
	pHexaNew[el[4]].m_iVertInd[7]=fac2vert[2];

	// the 7th new hexahedron
	pHexaNew[el[5]].m_iVertInd[0]=m_pHexas[i].m_iVertInd[6];
	pHexaNew[el[5]].m_iVertInd[1]=edg2vert[9];
	pHexaNew[el[5]].m_iVertInd[2]=fac2vert[5];
	pHexaNew[el[5]].m_iVertInd[3]=edg2vert[10];
						 
	pHexaNew[el[5]].m_iVertInd[4]=edg2vert[6];
	pHexaNew[el[5]].m_iVertInd[5]=fac2vert[2];
	pHexaNew[el[5]].m_iVertInd[6]=midEl;
	pHexaNew[el[5]].m_iVertInd[7]=fac2vert[3];

	// the 8th new hexahedron
	pHexaNew[el[6]].m_iVertInd[0]=m_pHexas[i].m_iVertInd[7];
	pHexaNew[el[6]].m_iVertInd[1]=edg2vert[10];
	pHexaNew[el[6]].m_iVertInd[2]=fac2vert[5];
	pHexaNew[el[6]].m_iVertInd[3]=edg2vert[11];
						 
	pHexaNew[el[6]].m_iVertInd[4]=edg2vert[7];
	pHexaNew[el[6]].m_iVertInd[5]=fac2vert[3];
	pHexaNew[el[6]].m_iVertInd[6]=midEl;
	pHexaNew[el[6]].m_iVertInd[7]=fac2vert[4];

	iOffset+=7;

  }//end for

  int *piVertAtBdrNew = new int[iNVTnew];
  for(int i=0;i<m_iNVT;i++)
  {
    piVertAtBdrNew[i]=m_piVertAtBdr[i];
  }

  //assign new vertex properties for edge midpoints
  for(int i=0;i<m_iNMT;i++)
  {
    int ivt1=this->m_VertAtEdge[i].m_iVertInd[0];
    int ivt2=this->m_VertAtEdge[i].m_iVertInd[1];
    
    if(m_piVertAtBdr[ivt1]==1 && m_piVertAtBdr[ivt2]==1)
    {
      piVertAtBdrNew[m_iNVT+i]=1;
    }
    else
    {
      piVertAtBdrNew[m_iNVT+i]=0;
    }
  }//end for  
  
  //assign new vertex properties for face midpoints
  for(int i=0;i<m_iNAT;i++)
  {
    int ivt1 = m_VertAtFace[i].m_iVertInd[0];
    int ivt2 = m_VertAtFace[i].m_iVertInd[1];
    int ivt3 = m_VertAtFace[i].m_iVertInd[2];
    int ivt4 = m_VertAtFace[i].m_iVertInd[3];
    
    if(m_piVertAtBdr[ivt1]==1 && m_piVertAtBdr[ivt2]==1 && m_piVertAtBdr[ivt3]==1 && m_piVertAtBdr[ivt4]==1)
    {
      piVertAtBdrNew[m_iNVT+m_iNMT+i]=1;            
    }
    else
    {
      piVertAtBdrNew[m_iNVT+m_iNMT+i]=0;      
    }
  }//end for
  
  for(int i=0;i<m_iNEL;i++)
  {
    piVertAtBdrNew[m_iNVT+m_iNMT+m_iNAT+i]=0;            
  }//end for    
  
  delete[] m_pVertexCoords;
  m_pVertexCoords = pVertexCoordsNew;
  delete[] m_pHexas;
  m_pHexas=pHexaNew;

  m_iNVT=iNVTnew;

  m_iNEL=iNELnew;

  delete[] m_myTraits;
  m_myTraits = new Traits[iNVTnew];
  
  delete[] m_piVertAtBdr;
  m_piVertAtBdr = piVertAtBdrNew;
  
};

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::VertAtBdr()
{
  
  int *VertAtBdr=new int[m_iNVT];
  
  for(int i=0;i<m_iNVT;i++)
  {
    VertAtBdr[i]=m_piVertAtBdr[i];
    m_piVertAtBdr[i]=0;
  }
  
  for(int i=0;i<m_iNEL;i++)
  {
    for(int j=0;j<6;j++)
    {
      if(m_pHexas[i].m_iAdj[j]==-1)
      {
      //m_piVertAtBdr[m_iNVT+m_iNMT+m_iNAT+i]=0;            
      //std::cout<<"Found boundary face... "<<std::endl;      
        int faceindex=m_pHexas[i].m_iFaces[j];      
        for(int k=0;k<4;k++)
        {
          if(VertAtBdr[m_VertAtFace[faceindex].m_iVertInd[k]]!=0)
            m_piVertAtBdr[m_VertAtFace[faceindex].m_iVertInd[k]]=1;                    
        }
      }
    }
  }//end for  

  delete[] VertAtBdr;
  
}//End VertAtBdr

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::InitStdMesh()
{
#ifdef WINDOWS
  CTimer timer;
  timer.Start();
#endif
  GenElAtVert();
#ifdef WINDOWS
  std::cout<<"Time for routine GenElAtVert(): "<<timer.Elapsed()<<std::endl;
  timer.Stop();
  timer.Start();
#endif
  GenNeighboursAtEl();
#ifdef WINDOWS
  std::cout<<"Time for routine GenNeighboursAtEl(): "<<timer.Elapsed()<<std::endl;
  timer.Stop();
  timer.Start();
#endif
  GenEdgesAtEl();
#ifdef WINDOWS
  std::cout<<"Time for routine GenEdgesAtEl(): "<<timer.Elapsed()<<std::endl;
  timer.Stop();
  timer.Start();
#endif
  GenFacesAtEl();
#ifdef WINDOWS
  std::cout<<"Time for routine GenFacesAtEl(): "<<timer.Elapsed()<<std::endl;
  timer.Stop();
  timer.Start();
#endif
  GenVertAtEdg();
#ifdef WINDOWS
  std::cout<<"Time for routine GenVertAtEdg(): "<<timer.Elapsed()<<std::endl;
  timer.Stop();
  timer.Start();
#endif
  GenVertAtFac();
#ifdef WINDOWS
  std::cout<<"Time for routine GenVertAtFac(): "<<timer.Elapsed()<<std::endl;
  timer.Stop();
  timer.Start();
#endif
	GenVertexVertex();
  
 if(m_iRefinementLevel==1)  
   VertAtBdr();

};

template<class T,class Traits>
void CUnstructuredGrid<T,Traits>::CleanExtended()
{
  
  if(this->m_piElAtVert)
  {
	delete[] m_piElAtVert;
	m_piElAtVert=NULL;
  }

  if(this->m_piElAtVertIdx)
  {
	delete[] m_piElAtVertIdx;
	m_piElAtVertIdx=NULL;
  }

  if(this->m_VertAtEdge)
  {
	delete[] m_VertAtEdge;
	m_VertAtEdge=NULL;
  }

  if(this->m_VertAtFace)
  {
	delete[] m_VertAtFace;
	m_VertAtFace=NULL;
  }

};

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CUnstructuredGrid<Real,DTraits>;

//----------------------------------------------------------------------------

}