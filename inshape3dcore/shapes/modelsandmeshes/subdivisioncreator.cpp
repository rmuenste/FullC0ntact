#include "subdivisioncreator.h"
#include <traits.h>
#include <boundingvolumetree3.h>
#include <deque>
#include <iostream>
#include <stdlib.h>
#include <distancetriangle.h>

namespace i3d {

CSubdivisionCreator::CSubdivisionCreator()
{

}

CSubdivisionCreator::~CSubdivisionCreator()
{
}

CSubdivisionCreator::CSubdivisionCreator(const CSubdivisionCreator &copy)
{
	this->m_pRessources = copy.m_pRessources;
}

void CSubdivisionCreator::Subdivide(CBoundingVolumeNode3<AABB3r,Real,CTraits> **&pNodes, CBoundingVolumeTree3<AABB3r, Real, CTraits, CSubdivisionCreator> &tree)
{
  using namespace std;
  //allocate memory for the children of the root
  //pNodes = new CBoundingVolumeNode3<AABB3r,Real,CTraits>*[1];

  int isize = pow(2.0,m_pRessources->m_iDepth+1)-1;
  inode_ = 0;

  tree.nodes_ = new CBoundingVolumeNode3<AABB3r,Real,CTraits>*[isize];
  pNodes = tree.nodes_;

  //queue data structures for the Top-Down tree construction
  //holds the AABBTree's nodes//
  std::deque< CBoundingVolumeNode3<AABB3r,Real,CTraits>* > qNodes;
  std::deque< CBoundingVolumeNode3<AABB3r,Real,CTraits>* > qNodesNextLevel;

  /* create the top level nodes in the hierarchy */
  for(int i = 0; i < 1; i++)
  {
    //insert the AABBTrees nodes into the queue
    //and construct circle tree nodes from them
    pNodes[i] = new CBoundingVolumeNode3<AABB3r,Real,CTraits>(this->m_pRessources->box);
    pNodes[i]->m_Traits.m_vTriangles=*(m_pRessources->m_pTriangles);
    pNodes[i]->m_BV.init(pNodes[i]->m_Traits.m_vTriangles);

    qNodes.push_back(pNodes[i]);
  }//end for

  CBoundingVolumeNode3<AABB3r,Real,CTraits> *pRoot = pNodes[0];

  while(!EvaluateTerminationCrit(pRoot))
  {

    /* Top-Down build of the tree */
    //build level by level
    while(!qNodes.empty())
    {

      //get the first element
      CBoundingVolumeNode3<AABB3r,Real,CTraits> *pNode = qNodes.front();

      //remove it from the queue
      qNodes.pop_front();

      if (pNode != NULL)
        SubdivideNode(pNode);

      if (pNode->m_Children[0] != NULL)
        qNodesNextLevel.push_back(pNode->m_Children[0]);

      if (pNode->m_Children[1] != NULL)
        qNodesNextLevel.push_back(pNode->m_Children[1]);

    }//end while

    //swap the queues
    qNodes=qNodesNextLevel;
    qNodesNextLevel.clear();

  }

}

void CSubdivisionCreator::Subdivide(CBoundingVolumeNode3<AABB3r,Real,CTraits> **&pNodes)
{
	using namespace std;
	//allocate memory for the children of the root
	pNodes = new CBoundingVolumeNode3<AABB3r,Real,CTraits>*[1];

	//queue data structures for the Top-Down tree construction
	//holds the AABBTree's nodes//
	std::deque< CBoundingVolumeNode3<AABB3r,Real,CTraits>* > qNodes;
	std::deque< CBoundingVolumeNode3<AABB3r,Real,CTraits>* > qNodesNextLevel;

	/* create the top level nodes in the hierarchy */
	for(int i = 0; i < 1; i++)
	{
		//insert the AABBTrees nodes into the queue
		//and construct circle tree nodes from them
		pNodes[i] = new CBoundingVolumeNode3<AABB3r,Real,CTraits>(this->m_pRessources->box);
		pNodes[i]->m_Traits.m_vTriangles=*(m_pRessources->m_pTriangles);
		pNodes[i]->m_BV.init(pNodes[i]->m_Traits.m_vTriangles);
    //ApproxUpperBound(pNodes[i]);
		//set the curve ID
		//m_Children[i]->SetID(iID);
		qNodes.push_back(pNodes[i]);
	}//end for
	
	CBoundingVolumeNode3<AABB3r,Real,CTraits> *pRoot = pNodes[0];


	while(!EvaluateTerminationCrit(pRoot))
	{
		//cout<<"Termination criterion: "<<EvaluateTerminationCrit(pRoot, 7)<<endl;
		/* Top-Down build of the tree */

    bool balanced = true;
		//build level by level
    for (auto &node : qNodes)
    {
      //cout<<"size queueNextLevel: "<<qNodesNextLevel.size()<<endl;
      if (node != NULL)
      {
        balanced = balanced & SubdivideNodeDry(node);
      }
    }

    if (!balanced)
      return;

		while(!qNodes.empty())
		{

			//get the first element
			CBoundingVolumeNode3<AABB3r,Real,CTraits> *pNode = qNodes.front();
			
			//remove it from the queue
			qNodes.pop_front();

      if (pNode != NULL)
        SubdivideNode(pNode);
			
      if (pNode->m_Children[0] != NULL)
			  qNodesNextLevel.push_back(pNode->m_Children[0]);

      if (pNode->m_Children[1] != NULL)
			  qNodesNextLevel.push_back(pNode->m_Children[1]);
			
		}//end while
		
		//cout<<"size queue before swap: "<<qNodes.size()<<endl;		
		//swap the queues
		qNodes=qNodesNextLevel;
		qNodesNextLevel.clear();
		//cout<<"size queue after swap: "<<qNodes.size()<<endl;				
	}
	
}

void CSubdivisionCreator::SubdivideNode(CBoundingVolumeNode3<AABB3r,Real,CTraits> *&pNode)
{

  if (pNode == NULL || pNode->m_Traits.m_vTriangles.size() <= 1)
  {
    return;
  }

	/* get the nodes bounding box */
	const AABB3r &bAABB3 = pNode->m_BV;

	/* get the longest axis the bounding volume will be split there */
	int iAxis    = bAABB3.longestAxis();

	/* get the center of the bounding volume */
	VECTOR3 vCenter = bAABB3.getCenter();

	std::vector<Triangle3r> &vTriangles = pNode->m_Traits.m_vTriangles;

  std::vector<Triangle3r> triangles0;
  std::vector<Triangle3r> triangles1;

	/* split the items into two buckets relative to the split axis */
	for(int i = 0; i < vTriangles.size(); i++)
	{
		Triangle3r &Tri = vTriangles[i];

		/* value at that the bounding volume is split along the split axis */
		if(Tri.GetCenter().m_dCoords[iAxis] < vCenter.m_dCoords[iAxis])
		{
      triangles0.push_back(Tri);
		}
		else
		{
      triangles1.push_back(Tri);
		}
	}//end for

/*	std::cout<<"Number of Triangles in Node 1: "<<pNode->m_Children[0]->m_Traits.m_vTriangles.size()<<std::endl;
	std::cout<<"Number of Triangles in Node 2: "<<pNode->m_Children[1]->m_Traits.m_vTriangles.size()<<std::endl;*/

  if (triangles0.size() > 0 && triangles1.size() > 0)
  {
    pNode->m_Children[0] = new CBoundingVolumeNode3<AABB3r, Real, CTraits>();
    pNode->m_Children[0]->m_Traits.m_vTriangles = triangles0;
    pNode->m_Children[0]->m_BV.init(pNode->m_Children[0]->m_Traits.m_vTriangles);

    pNode->m_Children[1] = new CBoundingVolumeNode3<AABB3r, Real, CTraits>();
    pNode->m_Children[1]->m_Traits.m_vTriangles = triangles1;
    pNode->m_Children[1]->m_BV.init(pNode->m_Children[1]->m_Traits.m_vTriangles);
  }
  else
  {
    std::cout<<"Function SubdivideNode: Number of triangles less than 1. Error."<<std::endl;
    std::cout<<"Unbalanced tree. Stopping subdivision."<<std::endl;
    exit(0);
  }

}//end SubdivideNode

bool CSubdivisionCreator::SubdivideNodeDry(CBoundingVolumeNode3<AABB3r, Real, CTraits> *&pNode)
{

  if (pNode == NULL || pNode->m_Traits.m_vTriangles.size() <= 1)
  {
    return false;
  }

  /* get the nodes bounding box */
  const AABB3r &bAABB3 = pNode->m_BV;

  /* get the longest axis the bounding volume will be split there */
  int iAxis = bAABB3.longestAxis();

  /* get the center of the bounding volume */
  VECTOR3 vCenter = bAABB3.getCenter();

  std::vector<Triangle3r> &vTriangles = pNode->m_Traits.m_vTriangles;

  int triangles0 = 0;
  int triangles1 = 0;

  /* split the items into two buckets relative to the split axis */
  for (int i = 0; i < vTriangles.size(); i++)
  {
    Triangle3r &Tri = vTriangles[i];

    /* value at that the bounding volume is split along the split axis */
    if (Tri.GetCenter().m_dCoords[iAxis] < vCenter.m_dCoords[iAxis])
    {
      triangles0++;
    }
    else
    {
      triangles1++;
    }
  }//end for

  return (triangles0 > 0 && triangles1 > 0);

}//end SubdivideNode

void CSubdivisionCreator::ApproxUpperBound(CBoundingVolumeNode3<AABB3r,Real,CTraits> *&pNode)
{

	/* get the nodes bounding box */
	const AABB3r &bAABB3 = pNode->m_BV;

	/* get the center of the bounding volume */
	VECTOR3 vCenter = bAABB3.getCenter();

	std::vector<Triangle3r> &vTriangles = pNode->m_Traits.m_vTriangles;

  CDistancePointTriangle<Real> distTri(vTriangles[0],vCenter);
  Real minDist = distTri.ComputeDistance();

  pNode->m_BV.upperLimit_ = distTri.m_vClosestPoint1;

	/* split the items into two buckets relative to the split axis */
	for(int i = 0; i < vTriangles.size(); i++)
	{
		Triangle3r &Tri = vTriangles[i];

    CDistancePointTriangle<Real> distTriangle(vTriangles[i],vCenter);
    Real dist = distTriangle.ComputeDistanceSqr();
    if(dist < minDist)
    {
      dist = minDist;
      pNode->m_BV.upperLimit_ = distTriangle.m_vClosestPoint1;
    }
  }
}

bool CSubdivisionCreator::EvaluateTerminationCrit(CBoundingVolumeNode3<AABB3r,Real,CTraits> *pNode)
{
	int depth = CBoundingVolumeNode3<AABB3r,Real,CTraits>::GetSubTreeDepth(pNode,0);
	if(depth >= m_pRessources->m_iDepth)
	{
		return true;
	}
	else
		return false;
}

void CSubdivisionCreator::SubdivideBox(CBoundingVolumeNode3<AABB3r,Real,CTraits> **&pNodes)
{
  using namespace std;
  //allocate memory for the children of the root
  pNodes = new CBoundingVolumeNode3<AABB3r,Real,CTraits>*[6];

  /* create the top level nodes in the hierarchy */
  for(int i = 0; i < 6; i++)
  {
    //insert the AABBTrees nodes into the queue
    //and construct circle tree nodes from them
    pNodes[i] = new CBoundingVolumeNode3<AABB3r,Real,CTraits>();
    pNodes[i]->m_Traits.m_vTriangles.push_back((*m_pRessources->m_pTriangles)[2*i]);
    pNodes[i]->m_Traits.m_vTriangles.push_back((*m_pRessources->m_pTriangles)[2*i+1]);
    pNodes[i]->m_BV.init(pNodes[i]->m_Traits.m_vTriangles);
  }//end for

  CBoundingVolumeNode3<AABB3r,Real,CTraits> *pRoot = pNodes[0];

}


}
