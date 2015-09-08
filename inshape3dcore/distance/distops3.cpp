/****************************************************************************
**
** Copyright (C) 2005-2007 Trolltech ASA. All rights reserved.
**
** This file is part of the example classes of the Qt Toolkit.
**
** This file may be used under the terms of the GNU General Public
** License version 2.0 as published by the Free Software Foundation
** and appearing in the file LICENSE.GPL included in the packaging of
** this file.  Please review the following information to ensure GNU
** General Public Licensing requirements will be met:
** http://www.trolltech.com/products/qt/opensource.html
**
** If you are unsure which license is appropriate for your use, please
** review the following information:
** http://www.trolltech.com/products/qt/licensing.html or contact the
** sales department at sales@trolltech.com.
**
** This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
** WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
**
****************************************************************************/
#include <limits>
#include <list>

#include "distops3.h"
#include "intersectorray3tri3.h"
#include "intersectoraabbray.h"
#include <set>
#include <iostream>
#include <distancetriangle.h>

namespace i3d {

//=====================================================================================
/*!
* This is the class constructor, yet there are no
* initialisations
*/
CDistOps3::CDistOps3(void)
{
}

//=====================================================================================
/*!
* Since we dont have any member variables
* we do not need deinitialisations
*/
CDistOps3::~CDistOps3(void)
{
}

//=====================================================================================
/*!
* Distance calculation by brute force this is supposed to be the
* slowest of all algorithms.
* \parameter CModel3D &model : reference to the 3D model object
* \parameter VECTOR3 &vQuery : reference to the current point of the grid
*/
Real CDistOps3::BruteForceDistance(Model3D &model, const Vector3f &vQuery) const
{
	using namespace std;

	//The return value
	Real distance;
	//helper variables
	VECTOR3 c;
	double d = std::numeric_limits<Real>::max();
	
	//initialize distance with default values
	distance  = std::numeric_limits<Real>::max();

  for(unsigned int i=0;i<model.meshes_.size();i++)
  {
    Mesh3D& mesh=model.meshes_[i];

    //init our intersector
    int nNumP =  mesh.numVerts_;
    //get a reference to the vertices of the 3d model
    const Vertex3Array& vVertices = mesh.getVertices();

	  CDynamicArray<TriFace>::iterator faceIter;
	  int j=0;
	  for(faceIter=mesh.faces_.begin();faceIter!=mesh.faces_.end();faceIter++)
	  {
      TriFace tri=*faceIter;

      //We loop through all triangular faces of the
      // model. This variable will hold the current face
      Triangle3<Real> tri3(mesh.getVertices()[tri[0]],mesh.getVertices()[tri[1]],mesh.getVertices()[tri[2]]);
      CDistancePointTriangle<Real> pointTriangle(tri3,vQuery);
      d = pointTriangle.ComputeDistance();
      if(d <distance)
      {
        distance = d;
      }
    }
  }

  //return the result
  return distance;

}//end BruteForceDistance

//=====================================================================================
/*!
* The brute force point classification algorithm, it does make
* use of any special data structures so it is the slowest
* but a parallel version exists.
*/
int CDistOps3::BruteForceInnerPoints(Model3D &model, const VECTOR3 &vQuery)
{

	//In this variable we count the number on intersections
	int nIntersections = 0;

	////Get the bounding box of the 3d model
	const AABB3r &rBox = model.GetBox();

	//Get the mesh
	Mesh3D& mesh0=model.meshes_[0];
	Vector3<Real> vTrans;
	//Transform into the coordinate system of the mesh
	vTrans=mesh0.TransfromWorldModelSingle(vQuery);

	//test if the query point is inside the
	//object's bounding box, if it is not inside
	//than we can return false
	if(!rBox.isPointInside(vTrans))
		return false;
	
	for(unsigned int i=0;i<model.meshes_.size();i++)
	{
	  Mesh3D& mesh=model.meshes_[i];
	  vTrans=mesh.TransfromWorldModelSingle(vQuery);
	  //initialise a ray with starting point vQuery along the x-axis
		Ray3<Real> ray3(vTrans,VECTOR3(0.9,0.8,0.02) );
	  CDynamicArray<TriFace>::iterator faceIter;
	  int j=0;
	  for(faceIter=mesh.faces_.begin();faceIter!=mesh.faces_.end();faceIter++)
	  {
		TriFace tri=*faceIter;

		//We loop through all triangular faces of the
		// model. This variable will hold the current face
		Triangle3<Real> tri3(mesh.getVertices()[tri[0]],mesh.getVertices()[tri[1]],mesh.getVertices()[tri[2]]);

		//init our intersector
		CIntersectorRay3Tri3<Real> intersector(ray3, tri3);
		
		//test for intersection
		if(intersector.Intersection())
			nIntersections++;
		j++;
	  }//end for
	}

	//if the number of intersection is even
	//we return false else true
	if(nIntersections % 2 == 0)
		return 0;
	else
		return 1;

}//end BruteForceInnerPoints

//=====================================================================================
	//* The brute force point classification algorithm, it does make
	//* use of any special data structures so it is the slowest
	//* but a parallel version exists.This routine should be used with 
	//* objects that do not move, i.e. static objects
	//*/
int CDistOps3::BruteForceInnerPointsStatic(const Model3D &model, const VECTOR3 &vQuery)
{

	//In this variable we count the number on intersections
	int nIntersections = 0;
	for(unsigned int i=0;i<model.meshes_.size();i++)
	{
	  const Mesh3D& mesh=model.meshes_[i];
		Ray3<Real> ray3(vQuery,VECTOR3(0.9,0.8,0.02) );
    //CRay3<Real> ray3(vQuery,VECTOR3(0.0,0.0,1.0) );
	  CDynamicArray<TriFace>::const_iterator faceIter;

		//Get the bounding box of the 3d model
		const AABB3r &rBox = mesh.getBox();
		//Get the mesh
		if(!rBox.isPointInside(vQuery))
			continue;
		
		//reset the number of intersection to zero for the current subobject
		nIntersections=0;
	  for(faceIter=mesh.faces_.begin();faceIter!=mesh.faces_.end();faceIter++)
	  {
		TriFace tri=*faceIter;
		//We loop through all triangular faces of the
		// model. This variable will hold the current face
		Triangle3<Real> tri3(mesh.GetVertices()[tri[0]],mesh.GetVertices()[tri[1]],mesh.GetVertices()[tri[2]]);
		//init our intersector
		CIntersectorRay3Tri3<Real> intersector(ray3, tri3);
		//test for intersection
		if(intersector.Intersection())
			nIntersections++;
	  }//end for faces
		//we finished looping through the faces of the subobject
		//look if the point is inside the subobject
		//if the number of intersection is even
		//we return false else true
		if(!(nIntersections % 2 == 0))
			return 1;

	}//end for meshes

	//The query point is not inside of any of the
	//submeshes
	return 0;
}//end BruteForceInnerPoints

//=====================================================================================
	//* The brute force point classification algorithm, it does make
	//* use of any special data structures so it is the slowest
	//* but a parallel version exists.This routine should be used with 
	//* objects that do not move, i.e. static objects
	//*/
int CDistOps3::BruteForcefbm(const Model3D &model, const VECTOR3 &vQuery, Ray3<Real> ray3)
{

	//In this variable we count the number on intersections
	int nIntersections = 0;
	for(unsigned int i=0;i<model.meshes_.size();i++)
	{
	  const Mesh3D& mesh=model.meshes_[i];
	  CDynamicArray<TriFace>::const_iterator faceIter;

		//Get the bounding box of the 3d model
		const AABB3r &rBox = mesh.getBox();
		//Get the mesh
		if(!rBox.isPointInside(vQuery))
			continue;
		
		//reset the number of intersection to zero for the current subobject
		nIntersections=0;
	  for(faceIter=mesh.faces_.begin();faceIter!=mesh.faces_.end();faceIter++)
	  {
		TriFace tri=*faceIter;
		//We loop through all triangular faces of the
		// model. This variable will hold the current face
		Triangle3<Real> tri3(mesh.GetVertices()[tri[0]],mesh.GetVertices()[tri[1]],mesh.GetVertices()[tri[2]]);
		//init our intersector
		CIntersectorRay3Tri3<Real> intersector(ray3, tri3);
		//test for intersection
		if(intersector.Intersection())
			nIntersections++;
	  }//end for faces
		//we finished looping through the faces of the subobject
		//look if the point is inside the subobject 
		//if the number of intersection is even
		//we return false else true
		if(!(nIntersections % 2 == 0))
			return 1;

	}//end for meshes

	//The query point is not inside of any of the
	//submeshes
	return 0;
}//end BruteForceInnerPoints


////=====================================================================================
///*!
//* The first version of the BAB type algorithms, this is just
//* the basic version without any special accerelation techniques.
//* \parameter AABBTree3f &tree : reference to the aabbtree datastructure used for BAB
//* \parameter VECTOR3 &vQuery  : the current point of the grid
//*/
//Real CDistOps3::SimpleLUB(AABBTree3f &tree, const VECTOR3 &vQuery)
//{
//
//	//variable declarations and initialisations
//	//=========================================
//	//helper variable we initialize it with the maximum possible value
//	double d = std::numeric_limits<double>::max();
//
//	//further helper variables
//	double lowerBound  = std::numeric_limits<double>::max();
//	double upperBound  = -std::numeric_limits<double>::max();
//
//	//In this variable we save the node of the AABBtree that
//	//is located closest to the query point
//	AABBNode3f *nBest = NULL;
//
//	//we need to count how many leaves of the
//	//AABBtree we have in our list
//	int nLeafCount = 0;
//	
//	//the list we need for the Breadth first search
//	//in the tree data structure
//	list<AABBNode3f*> lBFS;
//	list<AABBNode3f*>::iterator lIter;
//
//	//initialize this list with the children of the root
//	for(int i = 0; i < 2; i++)
//		lBFS.push_back(tree.GetRoot()->m_Children[i]);
//    
//	//get the current size of the list
//	int vSize = (int)lBFS.size();
//
//	//* loop until there are only leaves in the list */
//	while(vSize != nLeafCount)
//	{
//		//each time initialize with zeros
//		nLeafCount = 0;
//		int j = 0;
//
//		//each time set this to the maximum value
//		lowerBound = std::numeric_limits<double>::max();
//
//		//a auxilliary array so that we dont have to
//		//calculate these values multiple times
//		double *dLowerBounds = new double[vSize];
//
//		/* find best upper bound */
//		for(lIter = lBFS.begin(); lIter != lBFS.end(); lIter++)
//		{
//			AABBNode3f *pNode = *lIter;
//			dLowerBounds[j] = pNode->GetLowerBound(vQuery);
//			if(lowerBound > dLowerBounds[j])
//			{
//				lowerBound = dLowerBounds[j];
//				nBest = pNode;
//			}//end if
//			j++;
//		}//end for
//		
//		/* get upper bound for best element */
//		upperBound = nBest->GetUpperBound(vQuery);
//		
//		//now we check every element if we can prune
//		//it or if it has to remain
//		lIter = lBFS.begin();
//		for(int i = 0; i < vSize; i++)
//		{
//			//get the current element
//			AABBNode3f *pNode = *lIter;
//
//			//if the current element is the best element
//			//we replace it by its successors and we go on
//			if(pNode == nBest)
//			{
//				//if we have reached the leaf
//				//level no more refinement is possible
//				if(!pNode->IsLeaf())
//				{
//					lBFS.push_back(pNode->m_Children[0]);
//					lBFS.push_back(pNode->m_Children[1]);
//					lIter = lBFS.erase(lIter);
//					continue;
//				}//end if
//				//the node is a leaf, so we increase the leaf count
//				else
//				{
//					nLeafCount++;
//					lIter++;
//					continue;
//				}//end else
//			}//end if
//			
//			//we check if our upper bound on the distance
//			//is larger than the lower bound of the current node
//			//If the lower bound of the current node is smaller
//			//then we refine it
//			if(upperBound > dLowerBounds[i])
//			{
//				//is the node a leaf, then
//				// it can not be refined...
//				if(!pNode->IsLeaf())
//				{
//					lBFS.push_back(pNode->m_Children[0]);
//					lBFS.push_back(pNode->m_Children[1]);
//					lIter = lBFS.erase(lIter);
//				}//end if
//				else
//				{
//					nLeafCount++;
//					lIter++;
//				}
//			}//end if
//			//the node's lower bound is larger than
//			//the best upper bound, so it can be
//			//pruned away
//			else
//			{
//				lIter = lBFS.erase(lIter);
//			}//end else
//			
//		}//end for
//
//		//update the the current size of the list
//		vSize = (int)lBFS.size();
//
//		//delete the auxilliary array, so we dont make
//		//a memory leak
//		delete[] dLowerBounds;
//	}//end while
//
//	//get all the triangles contained in the best node
//	vector<CTriangle3f*>& vTriangles = nBest->GetTriangles();
//	//get all the vertices in the best node
//	vector<VECTOR3*> &vVerts = nBest->GetVertices();
//
//	//here we make a call to an auxilliary routine
//	//that calcs the distance the triangles of the
//	//best node
//	VECTOR3 vNearest;
//	d = DistTriPoint(vVerts, vQuery, vNearest);
//
//	//check all the remaining nodes for a possible 
//	//improvement
//	for(lIter = lBFS.begin(); lIter != lBFS.end(); lIter++)
//	{
//		AABBNode3f *pBox = *lIter;
//		if(pBox == nBest)
//			continue;
//
//		vector<VECTOR3*> &vVertices = pBox->GetVertices();
//
//		Real dist = DistTriPoint(vVertices, vQuery, vNearest);
//
//		if(dist < d)
//		{
//			d = dist;
//		}//end if
//
//	}//end for
//
//	//finally return the square root of the distance
//	return sqrt(d);
//
//}

////=====================================================================================
///*!
//* In this debug version of the simple BAB algorithm we calculate additional
//* output information, to provide better algorithm analysis.
//*/
//t_Collection CDistOps3::SimpleLUBDebug(AABBTree3f &tree, const VECTOR3 &vQuery)
//{
//	//variable declarations and initialisations
//	//=========================================
//	//helper variable we initialize it with the maximum possible value
//	double d = std::numeric_limits<double>::max();
//	
//	//the debug return structure
//	t_Collection sCollection;
//
//	//clear the collection
//	sCollection.lBFS.clear();
//
//	//further helper variables
//	double lowerBound  = std::numeric_limits<double>::max();
//	double upperBound  = -std::numeric_limits<double>::max();
//
//	//In this variable we save the node of the AABBtree that
//	//is located closest to the query point
//	AABBNode3f *nBest = NULL;
//
//	//we need to count how many leaves of the
//	//AABBtree we have in our list
//	int nLeafCount = 0;
//	
//	//the list we need for the Breadth first search
//	//in the tree data structure
//	list<AABBNode3f*> lBFS;
//	list<AABBNode3f*>::iterator lIter;
//
//	//initialize this list with the children of the root
//	for(int i = 0; i < 2; i++)
//		lBFS.push_back(tree.GetRoot()->m_Children[i]);
//    
//	//get the current size of the list
//	int vSize = (int)lBFS.size();
//
//	//* loop until there are only leaves in the list */
//	while(vSize != nLeafCount)
//	{
//		//each time initialize with zeros
//		nLeafCount = 0;
//		int j = 0;
//
//		//each time set this to the maximum value
//		lowerBound = std::numeric_limits<double>::max();
//
//		//a auxilliary array so that we dont have to
//		//calculate these values multiple times
//		double *dLowerBounds = new double[vSize];
//
//		/* find best upper bound */
//		for(lIter = lBFS.begin(); lIter != lBFS.end(); lIter++)
//		{
//			AABBNode3f *pNode = *lIter;
//			dLowerBounds[j] = pNode->GetLowerBound(vQuery);
//			if(lowerBound > dLowerBounds[j])
//			{
//				lowerBound = dLowerBounds[j];
//				nBest = pNode;
//			}//end if
//			j++;
//		}//end for
//		
//		/* get upper bound for best element */
//		upperBound = nBest->GetUpperBound(vQuery);
//		
//		//now we check every element if we can prune
//		//it or if it has to remain
//		lIter = lBFS.begin();
//		for(int i = 0; i < vSize; i++)
//		{
//			//get the current element
//			AABBNode3f *pNode = *lIter;
//
//			//if the current element is the best element
//			//we replace it by its successors and we go on
//			if(pNode == nBest)
//			{
//				//if we have reached the leaf
//				//level no more refinement is possible
//				if(!pNode->IsLeaf())
//				{
//					lBFS.push_back(pNode->m_Children[0]);
//					lBFS.push_back(pNode->m_Children[1]);
//					lIter = lBFS.erase(lIter);
//					continue;
//				}//end if
//				//the node is a leaf, so we increase the leaf count
//				else
//				{
//					nLeafCount++;
//					lIter++;
//					continue;
//				}//end else
//			}//end if
//			
//			//we check if our upper bound on the distance
//			//is larger than the lower bound of the current node
//			//If the lower bound of the current node is smaller
//			//then we refine it
//			if(upperBound > dLowerBounds[i])
//			{
//				//is the node a leaf, then
//				// it can not be refined...
//				if(!pNode->IsLeaf())
//				{
//					lBFS.push_back(pNode->m_Children[0]);
//					lBFS.push_back(pNode->m_Children[1]);
//					lIter = lBFS.erase(lIter);
//				}//end if
//				else
//				{
//					nLeafCount++;
//					lIter++;
//				}
//			}//end if
//			//the node's lower bound is larger than
//			//the best upper bound, so it can be
//			//pruned away
//			else
//			{
//				lIter = lBFS.erase(lIter);
//			}//end else
//			
//		}//end for
//
//		//update the the current size of the list
//		vSize = (int)lBFS.size();
//
//		//delete the auxilliary array, so we dont make
//		//a memory leak
//		delete[] dLowerBounds;
//	}//end while
//
//	//get all the triangles contained in the best node
//	vector<CTriangle3f*>& vTriangles = nBest->GetTriangles();
//	//get all the vertices in the best node
//	vector<VECTOR3*> &vVerts = nBest->GetVertices();
//
//	//here we make a call to an auxilliary routine
//	//that calcs the distance the triangles of the
//	//best node
//	VECTOR3 vNearest;
//	VECTOR3 vNearestTrial;
//	d = DistTriPoint(vVerts, vQuery, vNearest);
//	
//
//	//check all the remaining nodes for a possible 
//	//improvement
//	for(lIter = lBFS.begin(); lIter != lBFS.end(); lIter++)
//	{
//
//		AABBNode3f *pBox = *lIter;
//		sCollection.lBFS.push_back(pBox);
//		if(pBox == nBest)
//			continue;
//
//		vector<VECTOR3*> &vVertices = pBox->GetVertices();
//
//		Real dist = DistTriPoint(vVertices, vQuery, vNearestTrial);
//
//		if(dist < d)
//		{
//			d = dist;
//			nBest = pBox;
//			vNearest = vNearestTrial;
//		}//end if
//
//	}//end for
//
//	//assign the pointer in the debug structure
//	sCollection.pBestNode     = nBest;
//	sCollection.vClosestPoint = vNearest;
//
//
//	//assign square root of the distance
//	sCollection.rDistance =  sqrt(d);
//
//	return sCollection;
//
//}//end SimpleLUBDebug
//
////=====================================================================================
///*!
//* Here we have the BAB algorithm with the spatial coherency acceleration technique
//* \parameter AABBTree3f &tree : reference to the aabbtree data structure
//* \parameter VECTOR3 &vQuery  : reference to the current point of the grid
//* \Real rLUB                  : A bound on the distance, obtained by the preceeding vertex
//*/
//Res_t CDistOps3::CoherencyLUB(const AABBTree3f &tree, const VECTOR3 &vQuery, Real rLUB)
//{
//
//	Res_t result;
//
//	double d = std::numeric_limits<double>::max();
//
//	double lowerBound  = std::numeric_limits<double>::max();
//	double upperBound  = -std::numeric_limits<double>::max();
//
//	list<AABBNode3f*> lBFS;
//	list<AABBNode3f*>::iterator lIter;
//
//	for(int i = 0; i < 2; i++)
//		lBFS.push_back(tree.GetRoot()->m_Children[i]);
//    
//	AABBNode3f *nBest = NULL;
//
//	int vSize = (int)lBFS.size();
//
//	int nLeafCount = 0;
//
//	//* loop until there are only leaves in the list */
//
//	while(vSize != nLeafCount)
//	{
//		
//		nLeafCount = 0;
//		int j = 0;
//		lowerBound = std::numeric_limits<double>::max();
//		double *dLowerBounds = new double[vSize];
//		/* find best upper bound */
//		for(lIter = lBFS.begin(); lIter != lBFS.end(); lIter++)
//		{
//			AABBNode3f *pNode = *lIter;
//			dLowerBounds[j] = pNode->GetLowerBoundSqr(vQuery);
//			if(lowerBound > dLowerBounds[j])
//			{
//				lowerBound = dLowerBounds[j];
//				nBest = pNode;
//			}//end if
//			j++;
//		}//end for
//		
//		/* get upper bound for best element */
//		Real bestUpper = nBest->GetUpperBoundSqr(vQuery);
//		upperBound = (rLUB > bestUpper) ? bestUpper : rLUB;
//		
//		lIter = lBFS.begin();
//		for(int i = 0; i < vSize; i++)
//		{
//			
//			AABBNode3f *pNode = *lIter;
//			if(pNode == nBest)
//			{
//				if(!pNode->IsLeaf())
//				{
//					lBFS.push_back(pNode->m_Children[0]);
//					lBFS.push_back(pNode->m_Children[1]);
//					lIter = lBFS.erase(lIter);
//					continue;
//				}
//				else
//				{
//					nLeafCount++;
//					lIter++;
//					continue;
//				}
//			}//end if
//			
//			/* Can this subtree be pruned ? */
//			if(upperBound > dLowerBounds[i])
//			{
//				if(!pNode->IsLeaf())
//				{
//					lBFS.push_back(pNode->m_Children[0]);
//					lBFS.push_back(pNode->m_Children[1]);
//					lIter = lBFS.erase(lIter);
//				}//end if
//				else
//				{
//					nLeafCount++;
//					lIter++;
//				}
//			}//end if
//			else
//			{
//				lIter = lBFS.erase(lIter);
//			}//end else
//			
//		}//end for
//		vSize = (int)lBFS.size();
//		delete[] dLowerBounds;
//
//	}//end while
//
//	///* compute closest point in this circle */
//
//	vector<VECTOR3*> &vVerts = nBest->GetVertices();
//
//	VECTOR3 vNearest;
//
//	int i;
//
//	d = DistTriPoint(vVerts, vQuery, vNearest);
//
//	for(lIter = lBFS.begin(); lIter != lBFS.end(); lIter++)
//	{
//		AABBNode3f *pBox = *lIter;
//		if(pBox == nBest)
//			continue;
//
//		vector<VECTOR3*> &vVertices = pBox->GetVertices();
//
//		Real dist = DistTriPoint(vVertices, vQuery, vNearest);
//
//		if(dist < d)
//		{
//			d = dist;
//		}//end if
//
//	}//end for
//
//	result.rDistance = d;
//	result.vVec      = vNearest;
//
//	return result;
//}//end
//
//Real CDistOps3::DistTriPoint(vector<VECTOR3*>& vVerts, const VECTOR3 &vQuery, VECTOR3& vNearest)
//{
//	int i;
//
//	double d = std::numeric_limits<double>::max();
//	
//	int nNumP = (int)vVerts.size();
//
//	for(i = 0; i < nNumP; i++)
//	{
//
//		VECTOR3 vVec = *vVerts[i];
//
//		Real distance;
//		distance = VECTOR3::createVector(vVec, vQuery).norm2();
//
//		if(distance < d)
//		{
//			 d = distance;
//			 vNearest = vVec;
//		}//end if
//
//	}//end for
//
//	return d;
//}//end DistTriPoint
//
////=====================================================================================
///*!
//* The brute force point classification algorithm, it does make
//* use of any special data structures so it is the slowest
//* but a parallel version exists. This function has additional
//* output parameters that can be used for debugging.
//*/
//t_InnerPoints CDistOps3::DebugBruteForceInnerPoints(CModel3D &model, VECTOR3& vQuery)
//{
//	//We loop through all triangular faces of the
//	// model. This variable will hold the current face
//	CTriangle3f tri3;
//
//	//the return structure
//	t_InnerPoints tReturn;
//
//	//In this variable we count the number on intersections
//	int nIntersections = 0;
//
//	//Get the bounding box of the 3d model
//	const AABB3f &rBox = model.GetBox();
//
//	//initialise a ray with starting point vQuery along the x-axis
//	CRay3f ray3(vQuery,  VECTOR3(1,0,0) );
//
//	//test if the query point is inside the
//	//object's bounding box, if it is not inside
//	//than we can return false
//	if(!rBox.Inside(vQuery))
//	{
//		tReturn.InOut = false;
//		tReturn.Intersections = 0;
//		return tReturn;
//	}
//
//
//	//loop over all faces and test if there is a ray triangle intersection
//	for(int i = 0; i < model.GetFaces().size(); i++)
//	{
//		//get the next triangle
//		tri3 = *model.GetTriangle(i);
//
//		//init our intersector
//		CIntersectorRay3Tri3f intersector(ray3, tri3);
//		
//		//test for intersection
//		if(intersector.Intersection())
//			nIntersections++;
//	}//end for
//
//	//if the number of intersection is even
//	//we return false else true
//	if(nIntersections % 2 == 0)
//	{
//		tReturn.InOut = false;
//		tReturn.Intersections = nIntersections;
//		return tReturn;
//	}
//	else
//	{
//		tReturn.Intersections = nIntersections;
//		tReturn.InOut = true;
//		return tReturn;
//	}
//
//}//end DebugBruteForceInnerPoints
//
///*!
//* A more sophisticated method of calculating the inner points 
//*/
//bool CDistOps3::InnerPoints(AABBTree3f &tree, CModel3D & model, VECTOR3& vQuery)
//{
//
//	//We loop through all triangular faces of the
//	// model. This variable will hold the current face
//	CTriangle3f tri3;
//
//	//In this variable we count the number on intersections
//	int nIntersections = 0;
//
//	//Get the bounding box of the 3d model
//	const AABB3f &rBox = model.GetBox();
//
//	//initialise a ray with starting point vQuery along the x-axis
//	CRay3f ray3(vQuery,  VECTOR3(1,0,0) );
//
//	//test if the query point is inside the
//	//object's bounding box, if it is not inside
//	//than we can return false
//	if(!rBox.Inside(vQuery))
//		return false;
//
//	//the list we need for the Breadth first search
//	//in the tree data structure
//	list<AABBNode3f*> lBFS;
//	list<AABBNode3f*>::iterator lIter;
//
//	//initialize this list with the children of the root
//	for(int i = 0; i < 2; i++)
//		lBFS.push_back(tree.GetRoot()->m_Children[i]);
//    
//	//get the current size of the list
//	int vSize = (int)lBFS.size();
//
//	int nLeafCount = 0;
//	
//	lIter = lBFS.begin();
//	//* loop until there are only leaves in the list */
//	while(vSize != nLeafCount)
//	{
//		//each time initialize with zeros
//		nLeafCount = 0;
//		
//			//set the iterator to the beginning
//			lIter = lBFS.begin();
//			for(int i = 0; i < vSize; i++)
//			{
//
//				//get the current node
//				AABBNode3f *pNode = *lIter;
//
//				//check if the query point is inside
//				if(pNode->GetBV().Inside(vQuery))
//				{
//					//if the current node is not a leaf
//					//insert the children
//					if(!pNode->IsLeaf())
//					{
//						lBFS.push_back(pNode->m_Children[0]);
//						lBFS.push_back(pNode->m_Children[1]);
//						lIter = lBFS.erase(lIter);
//					}//end if
//					//the node is a leaf and the query point is inside
//					//we need to keep it
//					else
//					{
//						nLeafCount++;
//						lIter++;
//					}//end else
//				}//end if
//				//the query point is not inside
//				else
//				{
//					lIter = lBFS.erase(lIter);
//				}//end else
//
//			}//end for
//
//		//update the queue size
//		vSize = (int)lBFS.size();
//
//	}//end while
//
//	for(lIter = lBFS.begin(); lIter != lBFS.end(); lIter++)
//	{
//		
//		//get the current node
//		AABBNode3f *pNode = *lIter;
//		vector<CTriangle3f*> &vecTri = pNode->GetTriangles();
//
//		for(int j = 0; j < (int)vecTri.size(); j++)
//		{
//			//get the next triangle
//			tri3 = *vecTri[j];
//
//			//init our intersector
//			CIntersectorRay3Tri3f intersector(ray3, tri3);
//
//			//test for intersection
//			if(intersector.Intersection())
//				nIntersections++;
//
//		}//end for
//
//		//if the number of intersection is even
//		//we return false else true
//
//	}//end for
//
//	if(nIntersections % 2 != 0)
//		return true;
//	else
//		return false;
//
//}//end InnerPoints
//
///*!
//* A more sophisticated method of calculating the inner points 
//*/
//bool CDistOps3::InnerPointsBVH(AABBTree3f &tree, CModel3D & model, VECTOR3& vQuery)
//{
//
//	//We loop through all triangular faces of the
//	// model. This variable will hold the current face
//	CTriangle3f tri3;
//
//	//In this variable we count the number on intersections
//	int nIntersections = 0;
//
//	//Get the bounding box of the 3d model
//	const AABB3f &rBox = model.GetBox();
//
//
//	//initialise a ray with starting point vQuery along the x-axis
//	CRay3f ray3(vQuery,  VECTOR3(1,0,0) );
//
//	//test if the query point is inside the
//	//object's bounding box, if it is not inside
//	//than we can return false
//	if(!rBox.Inside(vQuery))
//		return false;
//
//	//the list we need for the Breadth first search
//	//in the tree data structure
//	list<AABBNode3f*> lBFS;
//	list<AABBNode3f*>::iterator lIter;
//
//	//initialize this list with the children of the root
//	for(int i = 0; i < 2; i++)
//		lBFS.push_back(tree.GetRoot()->m_Children[i]);
//    
//	//get the current size of the list
//	int vSize = (int)lBFS.size();
//
//	int nLeafCount = 0;
//	
//	lIter = lBFS.begin();
//	//* loop until there are only leaves in the list */
//	while(vSize != nLeafCount)
//	{
//		//each time initialize with zeros
//		nLeafCount = 0;
//		
//			//set the iterator to the beginning
//			lIter = lBFS.begin();
//			for(int i = 0; i < vSize; i++)
//			{
//
//
//				//get the current node
//				AABBNode3f *pNode = *lIter;
//
//				//Get an intersector to calculate the ray box intersection
//				CRay3f rRay(vQuery,  VECTOR3(1,0,0) );
//				CIntersectorAABBRay3f arIntersector(rRay,pNode->GetBV());
//
//
//				//our ray intersects with this box
//				if(arIntersector.Intersection())
//				{
//					//if the current node is not a leaf
//					//insert the children
//					if(!pNode->IsLeaf())
//					{
//						lBFS.push_back(pNode->m_Children[0]);
//						lBFS.push_back(pNode->m_Children[1]);
//						lIter = lBFS.erase(lIter);
//					}//end if
//					//the node is a leaf and the query point is inside
//					//we need to keep it
//					else
//					{
//						nLeafCount++;
//						lIter++;
//					}//end else
//				}//end if
//				//the query point is not inside
//				else
//				{
//					lIter = lBFS.erase(lIter);
//				}//end else
//
//			}//end for
//
//		//update the queue size
//		vSize = (int)lBFS.size();
//
//	}//end while
//
//	for(lIter = lBFS.begin(); lIter != lBFS.end(); lIter++)
//	{
//		
//		//get the current node
//		AABBNode3f *pNode = *lIter;
//		vector<CTriangle3f*> &vecTri = pNode->GetTriangles();
//
//		for(int j = 0; j < (int)vecTri.size(); j++)
//		{
//			//get the next triangle
//			tri3 = *vecTri[j];
//
//			//init our intersector
//			CIntersectorRay3Tri3f intersector(ray3, tri3);
//
//			//test for intersection
//			if(intersector.Intersection())
//				nIntersections++;
//
//		}//end for
//
//		//if the number of intersection is even
//		//we return false else true
//
//	}//end for
//
//	if(nIntersections % 2 != 0)
//		return true;
//	else
//		return false;
//
//}//end InnerPoints
//
///*!
//* A more sophisticated method of calculating the inner points 
//*/
//bool CDistOps3::InnerPointsList(AABBTree3f &tree, CModel3D & model, VECTOR3& vQuery)
//{
//
//	//We loop through all triangular faces of the
//	// model. This variable will hold the current face
//	CTriangle3f tri3;
//
//	//In this variable we count the number on intersections
//	int nIntersections = 0;
//
//	//Get the bounding box of the 3d model
//	const AABB3f &rBox = model.GetBox();
//
//
//	//initialise a ray with starting point vQuery along the x-axis
//	CRay3f ray3(vQuery,  VECTOR3(1,0,0) );
//
//	//test if the query point is inside the
//	//object's bounding box, if it is not inside
//	//than we can return false
//	if(!rBox.Inside(vQuery))
//		return false;
//
//	//the list we need for the Breadth first search
//	//in the tree data structure
//	list<AABBNode3f*> lBFS;
//	list<AABBNode3f*>::iterator lIter;
//
//
//	//get first element
//	lIter = tree.m_pLeaves.begin();
//	for(;lIter !=tree.m_pLeaves.end();lIter++)
//	{
//		//get the current node
//		AABBNode3f *pNode = *lIter;
//
//		//Get an intersector to calculate the ray box intersection
//		CRay3f rRay(vQuery,  VECTOR3(1,0,0) );
//		CIntersectorAABBRay3f arIntersector(rRay,pNode->GetBV());
//
//		//our ray intersects with this box
//		if(arIntersector.Intersection())
//		{
//			lBFS.push_back(pNode);
//		}//end if
//	}//end for
//
//	nIntersections = 0;
//
//	for(lIter = lBFS.begin(); lIter != lBFS.end(); lIter++)
//	{
//		
//		//get the current node
//		AABBNode3f *pNode = *lIter;
//		vector<CTriangle3f*> &vecTri = pNode->GetTriangles();
//
//		for(int j = 0; j < (int)vecTri.size(); j++)
//		{
//			//get the next triangle
//			tri3 = *vecTri[j];
//
//			//init our intersector
//			CIntersectorRay3Tri3f intersector(ray3, tri3);
//
//			//test for intersection
//			if(intersector.Intersection())
//				nIntersections++;
//
//		}//end for
//
//		//if the number of intersection is even
//		//we return false else true
//
//	}//end for
//
//	if(nIntersections % 2 != 0)
//		return true;
//	else
//		return false;
//
//}//end InnerPoints
//
///*!
//* The BVH
//*/
//bool CDistOps3::InnerPointsHPC(CModel3D &model, VECTOR3 &vQuery)
//{
//	return model.inside(vQuery);
//}//end InnerPointsHPC
//
///*!
//* A more sophisticated method of calculating the inner points 
//*/
//t_InnerCollection CDistOps3::InnerPointsListDebug(AABBTree3f &tree, CModel3D & model, VECTOR3& vQuery)
//{
//
//	//We loop through all triangular faces of the
//	// model. This variable will hold the current face
//	CTriangle3f tri3;
//
//	//In this variable we count the number on intersections
//	int nIntersections = 0;
//
//	//return structure
//	t_InnerCollection res;
//
//	set<CTriangle3f*> sTriangles;
//
//	//Get the bounding box of the 3d model
//	const AABB3f &rBox = model.GetBox();
//
//
//	//initialise a ray with starting point vQuery along the x-axis
//	CRay3f ray3(vQuery,  VECTOR3(1,0,0) );
//
//	//test if the query point is inside the
//	//object's bounding box, if it is not inside
//	//than we can return false
//	if(!rBox.Inside(vQuery))
//	{
//		res.bInOut = false;
//		res.nBoxesHit = 0;
//		res.nIntersections = 0;
//		return res;
//	}
//
//	//the list we need for the Breadth first search
//	//in the tree data structure
//	list<AABBNode3f*> lBFS;
//	list<AABBNode3f*>::const_iterator lIter1;
//	list<AABBNode3f*>::iterator lIter2;
//
//
//	//get first element
//	lIter1 = tree.GetLeaves().begin();
//	
//	for(;lIter1 !=tree.GetLeaves().end();lIter1++)
//	{
//		//get the current node
//		AABBNode3f *pNode = *lIter1;
//
//		//Get an intersector to calculate the ray box intersection
//		CRay3f rRay(vQuery,  VECTOR3(1,0,0) );
//		CIntersectorAABBRay3f arIntersector(rRay,pNode->GetBV());
//
//		//our ray intersects with this box
//		if(arIntersector.Intersection())
//		{
//			lBFS.push_back(pNode);
//		}//end if
//	}//end for
//
//	nIntersections = 0;
//	
//	for(lIter2 = lBFS.begin(); lIter2 != lBFS.end(); lIter2++)
//	{
//		//get the current node
//		AABBNode3f *pNode = *lIter2;
//		vector<CTriangle3f*> &vecTri = pNode->GetTriangles();
//
//		for(int j = 0; j < (int)vecTri.size(); j++)
//		{
//			sTriangles.insert(vecTri[j]);
//		}//end for
//	}//end for
//
//	set<CTriangle3f*>::iterator sIter;
//	for(sIter=sTriangles.begin(); sIter!=sTriangles.end(); sIter++)
//	{
//
//		tri3 = *(*sIter);
//
//		CIntersectorRay3Tri3f intersector(ray3, tri3);
//		//test for intersection
//		if(intersector.Intersection())
//			nIntersections++;
//
//	}//end for
//
//	//if the number of intersection is even
//	//we return false else true
//	if(nIntersections % 2 != 0)
//	{
//		res.bInOut = true;
//		res.nIntersections = nIntersections;
//		res.nBoxesHit = lBFS.size();
//		return res;
//	}
//	else
//	{
//		res.bInOut = false;
//		res.nBoxesHit = lBFS.size();
//		res.nIntersections = nIntersections;
//		return res;
//	}
//
//}//end InnerPoints
}
