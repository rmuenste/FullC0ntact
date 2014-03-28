/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#include "distancefuncgridmodel.h"
#include <vector>
#include <queue>
#include <distops3.h>
#include <compare.h>
#include <iostream>
#include <intersectorray3tri3.h>
#include <intersectoraabbray.h>

namespace i3d {

template<class T>
CDistanceFuncGridModel<T>::CDistanceFuncGridModel()
{
	
}
	 
template<class T>	 
CDistanceFuncGridModel<T>::CDistanceFuncGridModel(UnstructuredGrid<T,DTraits> *pGrid,const C3DModel &model): CDistanceFuncGrid<T>(pGrid)
{
	m_pModel = &model;
}

template<class T>
CDistanceFuncGridModel<T>::~CDistanceFuncGridModel(void )
{

}

template<class T>
void CDistanceFuncGridModel<T>::ComputeDistance()
{
    typename UnstructuredGrid<T,DTraits>::VertexIter vIter;
    typename UnstructuredGrid<T,DTraits>::EdgeIter   eIter;
		
		//now initialize the priority_queue and spread the distance information
		//the collision heap member variable
		std::priority_queue<DistQueueEntry,std::vector<DistQueueEntry> , CmpDist> distQueue;

		CDistOps3 op;
		const C3DModel &model = *m_pModel;
		vIter=CDistanceFuncGrid<T>::m_pGrid->VertexBegin();
		//classify all the points
		int i=0;
		std::cout<<"FBM start.. "<<distQueue.size()<<std::endl;
		for(;vIter!=CDistanceFuncGrid<T>::m_pGrid->VertexEnd();vIter++,i++)
		{
			CVector3<T> &vec = *vIter;
			int in=op.BruteForceInnerPointsStatic(model,vec);
			CDistanceFuncGrid<T>::m_pGrid->VertexTrait(i).iTag=in;
		}
		std::cout<<"FBM finish.. "<<distQueue.size()<<std::endl;
		
		eIter=CDistanceFuncGrid<T>::m_pGrid->EdgeBegin();
		i=0;
		//find the edges that have different inout tags
		for(;eIter!=CDistanceFuncGrid<T>::m_pGrid->EdgeEnd();eIter++,i++)
		{
			HexaEdge &edge = *eIter;
			int vertA   = edge.edgeVertexIndices_[0];
			int vertB   = edge.edgeVertexIndices_[1];
			int inA = CDistanceFuncGrid<T>::m_pGrid->VertexTrait(vertA).iTag;
			int inB = CDistanceFuncGrid<T>::m_pGrid->VertexTrait(vertB).iTag;
			if(inA != inB)
			{
				CDistanceFuncGrid<T>::m_pGrid->VertexTrait(vertA).distance = 1.0;
				CDistanceFuncGrid<T>::m_pGrid->VertexTrait(vertB).distance = 1.0;
				//approximate the distance by the half of the edge length
				CVector3<T> vAB = CDistanceFuncGrid<T>::m_pGrid->Vertex(vertB)-CDistanceFuncGrid<T>::m_pGrid->Vertex(vertA);
				CVector3<T> vRef = CDistanceFuncGrid<T>::m_pGrid->Vertex(vertA) + (vAB * 0.5);
				T dist = 0.5 * vAB.mag();
				
				CDistanceFuncGrid<T>::m_pGrid->VertexTrait(vertA).distance = dist;
				CDistanceFuncGrid<T>::m_pGrid->VertexTrait(vertB).distance = dist;

				CDistanceFuncGrid<T>::m_pGrid->VertexTrait(vertA).iX =  1;
				CDistanceFuncGrid<T>::m_pGrid->VertexTrait(vertB).iX =  1;

				//save the reference point
				CDistanceFuncGrid<T>::m_pGrid->VertexTrait(vertA).vRef     =  vRef;
				CDistanceFuncGrid<T>::m_pGrid->VertexTrait(vertB).vRef     =  vRef;
				
				//insert both vertices into the vertexQueue
				DistQueueEntry entryA;
				DistQueueEntry entryB;
				entryA.pTraits = &CDistanceFuncGrid<T>::m_pGrid->VertexTrait(vertA);
				entryA.vPoint  = &CDistanceFuncGrid<T>::m_pGrid->Vertex(vertA);
				entryA.iVert   = vertA;
				entryB.pTraits = &CDistanceFuncGrid<T>::m_pGrid->VertexTrait(vertB);
				entryB.vPoint  = &CDistanceFuncGrid<T>::m_pGrid->Vertex(vertB);
				entryB.iVert   = vertB;
				distQueue.push(entryA);
				distQueue.push(entryB);
			}
		}//end for edges
		
		while(!distQueue.empty())
		{
			DistQueueEntry entry = distQueue.top();
			distQueue.pop();
			typename UnstructuredGrid<T,DTraits>::VertexVertexIter vvIter;
			vvIter = CDistanceFuncGrid<T>::m_pGrid->VertexVertexBegin(entry.iVert);
			for(;vvIter!=CDistanceFuncGrid<T>::m_pGrid->VertexVertexEnd(entry.iVert);vvIter++)
			{
				DistQueueEntry entryNew;
				entryNew.pTraits = &CDistanceFuncGrid<T>::m_pGrid->VertexTrait(*vvIter);
				entryNew.vPoint  = &CDistanceFuncGrid<T>::m_pGrid->Vertex(*vvIter);
				entryNew.iVert   = *vvIter;
				CVector3<T> vAB = (*entry.vPoint-entry.pTraits->vRef);
				T dist = vAB.mag();
				if(entryNew.pTraits->iX == 1)
				{
					if(dist < entryNew.pTraits->distance)
					{
					entryNew.pTraits->distance=dist;
					entryNew.pTraits->vRef    = entry.pTraits->vRef;
					distQueue.push(entryNew);
					}
				}
				else
				{
					entryNew.pTraits->distance=dist;
					entryNew.pTraits->vRef    = entry.pTraits->vRef;
					entryNew.pTraits->iX = 1;
					distQueue.push(entryNew);
				}

			}
		}
}

template <class T>
int CDistanceFuncGridModel<T>::BruteForceInnerPointsStatic(const C3DModel &model, const CVector3<T> &vQuery)
{

	//In this variable we count the number on intersections
	int nIntersections = 0;
	for(unsigned int i=0;i<model.m_vMeshes.size();i++)
	{
	  const C3DMesh& mesh=model.m_vMeshes[i];
		Ray3<T> ray3(vQuery,VECTOR3(0.9,0.8,0.02) );
	  CDynamicArray<TriFace>::const_iterator faceIter;

		//Get the bounding box of the 3d model
		const AABB3<T> &rBox = mesh.GetBox();
		//Get the mesh
		if(!rBox.isPointInside(vQuery))
			continue;
		
		//reset the number of intersection to zero for the current subobject
		nIntersections=0;
	  for(faceIter=mesh.m_pFaces.begin();faceIter!=mesh.m_pFaces.end();faceIter++)
	  {
		TriFace tri=*faceIter;
		//We loop through all triangular faces of the
		// model. This variable will hold the current face
		Triangle3<T> tri3(mesh.GetVertices()[tri[0]],mesh.GetVertices()[tri[1]],mesh.GetVertices()[tri[2]]);
		//init our intersector
		CIntersectorRay3Tri3<T> intersector(ray3, tri3);
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

template<class T>
int CDistanceFuncGridModel<T>::PointInside(const CBoundingVolumeNode3<AABB3<T>,T,CTraits> *pNode, const CVector3<T> &vQuery)
{
  //needed world transformed triangles, world transformed BVH
  //world transformed triangles in BVH, BVH is in world space
  typename std::list<const CBoundingVolumeNode3<AABB3<T>,T,CTraits> *>::const_iterator i;
  //early out test
  //test point in bvh
  if(!pNode->m_BV.isPointInside(vQuery))
    return 0;

  //determine ray direction
  CVector3<T> dir(0.9,0.8,0.02);/// = vQuery - pNode->m_BV.GetCenter();

  //CRay3(const CVector3<T> &vOrig, const CVector3<T> &vDir);
  Ray3<T> ray(vQuery,dir);

  Traverse(pNode,ray);

  //loop through nodes and test for intersection 
  //with the contained triangles
  i=m_pNodes.begin();
  int nIntersections=0;
  for(;i!=m_pNodes.end();i++)
  {
    const CBoundingVolumeNode3<AABB3<T>,T,CTraits> *node = *i;
    typename std::vector<Triangle3<T> >::const_iterator j=node->m_Traits.m_vTriangles.begin();
    for(;j!=node->m_Traits.m_vTriangles.end();j++)
    {
      const Triangle3<T> &tri3 = *j;
      CIntersectorRay3Tri3<T> intersector(ray, tri3);
      //test for intersection
      if(intersector.Intersection())
        nIntersections++;
    }//end for j
  }//end for i
  //we finished looping through the triangles
  //if the number of intersection is even
  //we return false else true
  if(!(nIntersections % 2 == 0))
  {
    m_pNodes.clear();
    return 1;
  }
  else
  {
    m_pNodes.clear();
    return 0;
  }
}

template<class T>
void CDistanceFuncGridModel<T>::Traverse(const CBoundingVolumeNode3<AABB3<T>,T,CTraits> *pNode, const Ray3<T> &rRay)
{

  if(pNode->IsLeaf())
  {
    //add node to list of nodes
    this->m_pNodes.push_back(pNode);
    return;
  }

  CIntersectorAABBRay3<T> intersector0(rRay,pNode->m_Children[0]->m_BV);
  CIntersectorAABBRay3<T> intersector1(rRay,pNode->m_Children[1]->m_BV);

  if(intersector0.Intersection())
    Traverse(pNode->m_Children[0],rRay);

  if(intersector1.Intersection())
    Traverse(pNode->m_Children[1],rRay);

}


//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceFuncGridModel<Real>;
//----------------------------------------------------------------------------

}
