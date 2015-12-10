
///==============================================
///	      CBoundingVolumeNode3 Functions
///==============================================

template<class BV, class T, class Traits>
CBoundingVolumeNode3<BV, T, Traits>::CBoundingVolumeNode3()
{

	m_Children[0] = 0;
	m_Children[1] = 0;

}//end constructor

template<class BV, class T, class Traits>
CBoundingVolumeNode3<BV, T, Traits>::CBoundingVolumeNode3(const BV &rBV)
{
	m_Children[0] = 0;
	m_Children[1] = 0;
	m_BV = rBV;
}//end constructor

template<class BV, class T, class Traits>
CBoundingVolumeNode3<BV, T, Traits>::~CBoundingVolumeNode3()
{

	if(m_Children[0])
	{
		delete m_Children[0];
		m_Children[0] = NULL;
	}//end if

	if(m_Children[1])
	{
		delete m_Children[1];
		m_Children[1] = NULL;
	}//end if
}//end deconstructor

//template<class BV, class T, class Traits>
//static int CBoundingVolumeNode3<BV, T, Traits>::GetDepth(CBoundingVolumeNode3<BV,T,Traits> *pNode, int iDepth)
//{
//
//	int dR = 0;
//	int dL = 0;
//	int iReturn = 0;
//
//	if(!pNode->IsLeaf())
//	{
//		dL = GetDepth(pNode->m_Children[0],iDepth+1);
//		dR = GetDepth(pNode->m_Children[1],iDepth+1);
//		iReturn = ( dL >= dR ) ? dL : dR;
//		return iReturn;
//	}//end if
//	else
//	{
//		iReturn = iDepth;
//		return iReturn;
//	}
//
//	return iReturn;
//
//}//end DepthFirstSearch

///==============================================
///	      CBoundingVolumeTree3 Functions
///==============================================

template<class BV, class T, class Traits, class SD>
CBoundingVolumeTree3<BV, T, Traits, SD>::CBoundingVolumeTree3() : m_pSD(nullptr), m_Children(nullptr), m_iNumChildren(0)
{

}//end constructor

template<class BV, class T, class Traits, class SD>
CBoundingVolumeTree3<BV, T, Traits, SD>::~CBoundingVolumeTree3()
{

	DestroyTree();
	if(m_iNumChildren != 0)
	{
		delete[] m_Children;
		m_iNumChildren = 0;
	}
}//end deconstructor

template<class BV, class T, class Traits, class SD>
void CBoundingVolumeTree3<BV, T, Traits, SD>::DeleteSubTree(CBoundingVolumeNode3<BV,T,Traits> *pNode)
{

	if(pNode != NULL)
	{
		DeleteSubTree( pNode->m_Children[0] );
		pNode->m_Children[0] = NULL;
		DeleteSubTree( pNode->m_Children[1] );
		pNode->m_Children[1] = NULL;
		delete pNode;
		pNode = NULL;
	}//end if

}//end

template<class BV, class T, class Traits, class SD>
void CBoundingVolumeTree3<BV, T, Traits, SD>::DestroyTree()
{

	for(int i = 0; i < m_iNumChildren; i++)
	{
		DeleteSubTree(m_Children[i]);
	}//end for


}//end destroy tree

template<class BV, class T, class Traits, class SD>
void CBoundingVolumeTree3<BV, T, Traits, SD>::DestroyAndRebuilt(SD *pSD)
{
	
	DestroyTree();
	if(m_iNumChildren != 0)
	{
		delete[] m_Children;
		m_iNumChildren = 0;
	}	
	
	m_pSD=pSD;
	m_pSD->Subdivide(m_Children);
	m_iNumChildren=1;
	
}

template<class BV, class T, class Traits, class SD>
void CBoundingVolumeTree3<BV, T, Traits, SD>::InitTree()
{
	m_pSD->Subdivide(m_Children);
}//end InitTree

template<class BV, class T, class Traits, class SD>
void CBoundingVolumeTree3<BV, T, Traits, SD>::InitTree(SD *pSD)
{
	m_pSD=pSD;
	m_pSD->Subdivide(m_Children);
	m_iNumChildren=1;
}//end InitTree

template<class BV, class T, class Traits, class SD>
void CBoundingVolumeTree3<BV, T, Traits, SD>::BreadthFirstSearch()
{
	
	using namespace std;
	queue< CBoundingVolumeNode3<BV, T, Traits>* > qBFS;

	qBFS.push(m_Children[0]);

	while(!qBFS.empty())
	{
		CBoundingVolumeNode3<BV, T, Traits>* pNode = qBFS.front();
		std::cout<<"Adress: "<<pNode<<" Trait: "<<pNode->m_Traits.iCollision<<std::endl;
		qBFS.pop();
		if(!pNode->IsLeaf())
		{
			qBFS.push(pNode->m_Children[0]);
			qBFS.push(pNode->m_Children[1]);
		}//end if

	}//end while

}//end BreadthFirstSearch

//calculates the depth of Subtree with node pNode
template<class BV, class T, class Traits, class SD>
int CBoundingVolumeTree3<BV, T, Traits, SD>::CalcSubTreeDepth(CBoundingVolumeNode3<BV,T,Traits> *pNode, int iDepth)
{
	int dR = 0;
	int dL = 0;
	int iReturn = 0;
	
	if(!pNode->IsLeaf())
	{
		dL = CalcSubTreeDepth(pNode->m_Children[0],iDepth+1);
		dR = CalcSubTreeDepth(pNode->m_Children[1],iDepth+1);
		iReturn = ( dL >= dR ) ? dL : dR;
		return iReturn;
	}//end if
	else
	{
		iReturn = iDepth;
		return iReturn;
	}
	return iReturn;	
}

template<class BV, class T, class Traits, class SD>
std::vector<CBoundingVolumeNode3<BV, T, Traits> *> CBoundingVolumeTree3<BV, T, Traits, SD>::getLeaves()
{

  std::vector<CBoundingVolumeNode3<BV, T, Traits> *> leaves;
  CBoundingVolumeNode3<BV, T, Traits> *pNode0 = m_Children[0];
  CBoundingVolumeNode3<BV, T, Traits> *pNode1 = m_Children[1];

  if (m_Children[0] != NULL)
    searchLeaves(m_Children[0],leaves);

  return leaves;

}

//calculates the depth of Subtree with node pNode
template<class BV, class T, class Traits, class SD>
void CBoundingVolumeTree3<BV, T, Traits, SD>::searchLeaves(CBoundingVolumeNode3<BV, T, Traits> *pNode, std::vector<CBoundingVolumeNode3<BV, T, Traits> *> &leaves)
{

  if (!pNode->IsLeaf())
  {
    searchLeaves(pNode->m_Children[0], leaves);
    searchLeaves(pNode->m_Children[1], leaves);
  }//end if
  else
  {
    leaves.push_back(pNode);
  }

}

template<class BV, class T, class Traits, class SD>
int CBoundingVolumeTree3<BV, T, Traits, SD>::GetDepth()
{
	int iDepth;
	int itempDepth;
	if(m_iNumChildren==0)
	{
		return -1;
	}
	
	iDepth=CalcSubTreeDepth(m_Children[0],0);
	for(int i=1;i<m_iNumChildren;i++)
	{
		if((itempDepth=CalcSubTreeDepth(m_Children[i],0))>iDepth)
			iDepth=itempDepth;
	}
	return iDepth;
}

//return a vector containing the nodes at the specified level
template<class BV, class T, class Traits, class SD>
std::vector<CBoundingVolumeNode3<BV,T,Traits> *> CBoundingVolumeTree3<BV, T, Traits, SD>:: GetNodesLevel(int iLevel)
{

	int iDepth = GetDepth();
	
	std::vector<CBoundingVolumeNode3<BV,T,Traits> *> vec;
	std::vector<CBoundingVolumeNode3<BV,T,Traits> *> vecNext;
	
	if(iLevel > iDepth)
	{

		return vec;
	}
	
	if(iLevel==0)
	{
		for(int i=0;i<m_iNumChildren;i++)
		{
      if (m_Children[i] != NULL)
			  vec.push_back(m_Children[i]);
		}
		
		return vec;
		
	}//end if
	
	//push the children of the root

	for(int i=0;i<m_iNumChildren;i++)
	{
    if (m_Children[i] != NULL)
		  vec.push_back(m_Children[i]);
	}


	for(int i=1;i<=iLevel;i++)
	{
		//std::vector<CBoundingVolumeNode3<BV,T,Traits> *>::iterator vIter; is a nested dependent type
		//at parse time the parser/compiler does not what is BV,T,... and hence does not know that iterator is a type
		//
		typename std::vector<CBoundingVolumeNode3<BV,T,Traits> *>::iterator vIter;

		for(vIter=vec.begin();vIter!=vec.end();vIter++)
		{
			CBoundingVolumeNode3<BV, T, Traits>* pNode = *vIter;
      if (pNode->m_Children[0] != NULL && pNode->m_Children[0]->m_Traits.m_vTriangles.size() != 0)
      {
        //printf("number of triangles: %i level %i\n", pNode->m_Children[0]->m_Traits.m_vTriangles.size(),i);
        //if ()
        //{
        //  printf("Not so good\n");
        //}
        vecNext.push_back(pNode->m_Children[0]);
      }

      if (pNode->m_Children[1] != NULL  && pNode->m_Children[1]->m_Traits.m_vTriangles.size() != 0)
      {
        //printf("number of triangles: %i level %i\n", pNode->m_Children[1]->m_Traits.m_vTriangles.size(),i);
        //if (pNode->m_Children[1]->m_Traits.m_vTriangles.size() == 0)
        //{
        //  printf("Not so good\n");
        //}
        vecNext.push_back(pNode->m_Children[1]);
      }

		}//end for
		
		vec=vecNext;
		vecNext.clear();
	
	}//end for
	
	return vec;
	
}//end GetNodesLevel

//return a vector containing the nodes at the specified level
template<class BV, class T, class Traits, class SD>
void CBoundingVolumeTree3<BV, T, Traits, SD>:: GenTreeStatistics()
{
	using namespace std;
	int iNodesTotal=0;
	cout<<"==================================================="<<endl;
	cout<<"Printing tree statistics:"<<endl;

	int iDepth=GetDepth();
	cout<<"Depth: "<<iDepth<<endl;

	cout<<"#Nodes at Level:"<<endl;
	cout<<"----------------"<<endl;
	for(int i=0;i<=iDepth;i++)
	{
		std::vector<CBoundingVolumeNode3<BV,T,Traits> *> vec = GetNodesLevel(i);
		cout<<" Level "<<i<<" : "<<vec.size()<<endl;
		iNodesTotal+=vec.size();
	}
	cout<<"----------------"<<endl;
	cout<<"Total number of nodes: "<<iNodesTotal<<endl;
	cout<<"==================================================="<<endl;

}

template<class BV, class T, class Traits, class SD>
void CBoundingVolumeTree3<BV, T, Traits, SD>::InitBoxTree(SD *pSD)
{
  m_pSD=pSD;
  m_pSD->SubdivideBox(m_Children);
  m_iNumChildren=6;
}

