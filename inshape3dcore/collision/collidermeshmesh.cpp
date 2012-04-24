/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/ 


//===================================================
//                     INCLUDES
//===================================================


#include "collidermeshmesh.h"
#include <triangulator.h>
#include <meshobject.h>
#include <distancetriangletriangle.h>
#include <perftimer.h>
#include <aabb3.h>
#include <distancemeshmesh.h>
#include "collisioninfo.h"

namespace i3d {

CColliderMeshMesh::CColliderMeshMesh() 
{

}

CColliderMeshMesh::~CColliderMeshMesh() 
{

}

/**
* @see CCollider::Collide
*
*/  
void CColliderMeshMesh::Collide(CRigidBody *pBody0, CRigidBody *pBody1, std::vector<CContact> &vContacts, Real dDeltaT)
{
  CCollider::Collide(pBody0,pBody1,vContacts,dDeltaT);
}

/**
* @see CCollider::Collide
*
*/  
void CColliderMeshMesh::Collide(std::vector<CContact> &vContacts, Real dDeltaT)
{

  if(!m_pBody0->m_bAffectedByGravity && !m_pBody1->m_bAffectedByGravity)
    return;

  CMeshObject<Real> *pObject0 = dynamic_cast<CMeshObject<Real>* >(m_pBody0->m_pShape);
  CMeshObject<Real> *pObject1 = new CMeshObject<Real>();
  Real eps=0.0025;
  Real eps2=eps*eps;
	//front faces  [0,1]
	//back faces   [2,3]
	//bottom faces [4,5]
	//top faces    [6,7]
	//left faces   [8,9]
	//right faces  [10,11]
  VECTOR3 vNormals[]={VECTOR3(0,-1,0),VECTOR3(0,1,0),VECTOR3(0,0,-1),VECTOR3(0,0,1),VECTOR3(-1,0,0),VECTOR3(1,0,0)};

  //check if we have to convert shapes
  if(m_pBody1->m_iShape==CRigidBody::BOX)
  {
    //convert box2mesh
	  CRigidBody &body = *(m_pBody1);
    //triangulate and world tranform the box
		CTriangulator<Real, COBB3<Real> > triangulator;
		COBB3r *pBox = dynamic_cast<COBB3r*>(body.m_pShape);
    pObject1->m_Model=triangulator.Triangulate(*pBox);
    pObject1->m_Model.m_vMeshes[0].m_matTransform =body.GetTransformationMatrix();
		pObject1->m_Model.m_vMeshes[0].m_vOrigin =body.m_vCOM;
		pObject1->m_Model.m_vMeshes[0].TransformModelWorld();
    pObject1->m_Model.GenerateBoundingBox();
    pObject1->m_Model.m_vMeshes[0].GenerateBoundingBox();
    pObject1->m_Model.m_vMeshes[0].m_vOrigin = body.m_vCOM;
    std::vector<CTriangle3r> triangles = pObject1->m_Model.GenTriangleVector();

    CSubDivRessources myRessources(6,1,1,pObject1->m_Model.GetBox(),&triangles);
    CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
    pObject1->m_BVH.InitBoxTree(&subdivider);
  }//end 

  //calculate distance between the AABBs
  CAABB3r box0 = pObject0->m_BVH.GetChild(0)->m_BV; // GetAABB();
  CAABB3r box1 = pObject1->GetAABB();
  CPerfTimer timer;
  timer.Start();

  Real sqrDist=0;
  VECTOR3 vD;
  for(int j=0;j<3;j++)
  {
    if(box1.m_Verts[1].m_dCoords[j] < box0.m_Verts[0].m_dCoords[j])
    {
      vD.m_dCoords[j]= box1.m_Verts[1].m_dCoords[j] - box0.m_Verts[0].m_dCoords[j];
    }
    else if(box1.m_Verts[0].m_dCoords[j] > box0.m_Verts[1].m_dCoords[j])
    {
      vD.m_dCoords[j]= box1.m_Verts[0].m_dCoords[j] - box0.m_Verts[1].m_dCoords[j];
    }
    else
    {
      vD.m_dCoords[j]=0;
    }

    sqrDist+=vD.m_dCoords[j]*vD.m_dCoords[j];
  }//end for

  if(sqrDist > eps2)
  {
    return;
  }

  CDistanceMeshMesh<Real> distMeshMesh(&pObject0->m_BVH, &pObject1->m_BVH);

  distMeshMesh.ComputeDistanceEps(eps);

  std::list<std::pair<CBoundingVolumeNode3<CAABB3r,Real,CTraits>*,
    CBoundingVolumeNode3<CAABB3r,Real,CTraits>* > >::iterator iter=distMeshMesh.pairs.begin();

  for(;iter!=distMeshMesh.pairs.end();iter++)
  {
    CBoundingVolumeNode3<CAABB3r,Real,CTraits> *pNode0=iter->first;
    CBoundingVolumeNode3<CAABB3r,Real,CTraits> *pNode1=iter->second;
    std::vector<CTriangle3<Real> > &triangleMesh0 = pNode0->m_Traits.m_vTriangles;
    std::vector<CTriangle3<Real> > &triangleMesh1 = pNode1->m_Traits.m_vTriangles;

    for(int i=0;i<triangleMesh0.size();i++)
    {
      CTriangle3r &tri3_0 = triangleMesh0[i];
      for(int j=0;j<triangleMesh1.size();j++)
      {
        CTriangle3r &tri3_1 = triangleMesh1[j];

        CDistanceTriangleTriangle<Real> distTriangleTriangle(tri3_0,tri3_1);
        Real dist = distTriangleTriangle.ComputeDistanceSqr();

        //if the distance is smaller eps
        if(dist < eps2)
        {

          //compute stuff rel. velocity and add
          VECTOR3 &vPoint = distTriangleTriangle.m_vClosestPoint0;

          //compute the relative velocity
          VECTOR3 angPart = (VECTOR3::Cross(m_pBody0->GetAngVel(),vPoint-m_pBody0->m_vCOM));
          VECTOR3 relativeVelocity = (m_pBody0->m_vVelocity + angPart);

          //calculate a good normal
          VECTOR3 normal = distTriangleTriangle.m_vClosestPoint0 - distTriangleTriangle.m_vClosestPoint1;
          normal.Normalize();

          //relative velocity along the normal
          Real normalVelocity = relativeVelocity * normal;

          //check whether there will be a collision next time step
          if(normalVelocity < 0.0)
          {
            //std::cout<<"Pre-contact normal velocity: "<<relVel<<" colliding contact"<<std::endl;
            CContact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = normal;
            contact.m_vPosition0 = vPoint;
            contact.m_vPosition1 = vPoint;
            contact.m_pBody0     = m_pBody0;
            contact.m_pBody1     = m_pBody1;
            contact.id0          = contact.m_pBody0->m_iID;
            contact.id1          = contact.m_pBody1->m_iID;
            contact.vn           = normalVelocity;
            contact.m_iState     = CCollisionInfo::COLLIDING;            
            vContacts.push_back(contact);
          }//end if(relVel < 0.0)
          else if(normalVelocity < 0.00001)
          {
            CContact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = normal;
            contact.m_vPosition0 = vPoint;
            contact.m_vPosition1 = vPoint;
            contact.m_pBody0     = m_pBody0;
            contact.m_pBody1     = m_pBody1;
            contact.id0          = contact.m_pBody0->m_iID;
            contact.id1          = contact.m_pBody1->m_iID;
            contact.vn           = normalVelocity;
            contact.m_iState     = CCollisionInfo::TOUCHING; 
            vContacts.push_back(contact);
          }//end else if
          else
          {
            CContact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = normal;
            contact.m_vPosition0 = vPoint;
            contact.m_vPosition1 = vPoint;
            contact.m_pBody0     = m_pBody0;
            contact.m_pBody1     = m_pBody1;
            contact.id0          = contact.m_pBody0->m_iID;
            contact.id1          = contact.m_pBody1->m_iID;
            contact.vn           = normalVelocity;
            contact.m_iState     = CCollisionInfo::VANISHING_CLOSEPROXIMITY;            
            vContacts.push_back(contact);            
          }
        }//end if dist < eps

      }//end for j
    }//end for i
  }//end for iter

//  std::cout<<"Time triangle distance: "<<timer.GetTime()<<std::endl;
  delete pObject1;
}

//void CColliderMeshMesh::Collide(std::vector<CContact> &vContacts, Real dDeltaT)
//{
//  CMeshObject<Real> *pObject0 = dynamic_cast<CMeshObject<Real>* >(m_pBody0->m_pShape);
//  CMeshObject<Real> *pObject1 = new CMeshObject<Real>();
//
//	//front faces  [0,1]
//	//back faces   [2,3]
//	//bottom faces [4,5]
//	//top faces    [6,7]
//	//left faces   [8,9]
//	//right faces  [10,11]
//  VECTOR3 vNormals[]={VECTOR3(0,-1,0),VECTOR3(0,1,0),VECTOR3(0,0,1),VECTOR3(0,0,-1),VECTOR3(1,0,0),VECTOR3(-1,0,0)};
//
//  //check if we have to convert shapes
//  if(m_pBody1->m_iShape==CRigidBody::BOX)
//  {
//    //convert box2mesh
//	  CRigidBody &body = *(m_pBody1);
//    //triangulate and world tranform the box
//		CTriangulator<Real, COBB3<Real> > triangulator;
//		COBB3r *pBox = dynamic_cast<COBB3r*>(body.m_pShape);
//    pObject1->m_Model=triangulator.Triangulate(*pBox);
//    pObject1->m_Model.m_vMeshes[0].m_matTransform =body.m_matTransform;
//		pObject1->m_Model.m_vMeshes[0].m_vOrigin =body.m_vCOM;
//		pObject1->m_Model.m_vMeshes[0].TransformModelWorld();
//    pObject1->m_Model.GenerateBoundingBox();
//    pObject1->m_Model.m_vMeshes[0].GenerateBoundingBox();
//    pObject1->m_Model.m_vMeshes[0].m_vOrigin = body.m_vCOM;
//    std::vector<CTriangle3r> triangles = pObject1->m_Model.GenTriangleVector();
//
//    CSubDivRessources myRessources(6,1,1,pObject1->m_Model.GetBox(),&triangles);
//    CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
//    pObject1->m_BVH.InitBoxTree(&subdivider);
//  }//end 
//
//  //calculate distance between the AABBs
//  CAABB3r box0 = pObject0->m_BVH.GetChild(0)->m_BV; // GetAABB();
//  CAABB3r box1 = pObject1->GetAABB();
//  CPerfTimer timer;
//  timer.Start();
//
//  Real sqrDist=0;
//  VECTOR3 vD;
//  for(int j=0;j<3;j++)
//  {
//    if(box1.m_Verts[1].m_dCoords[j] < box0.m_Verts[0].m_dCoords[j])
//    {
//      vD.m_dCoords[j]= box1.m_Verts[1].m_dCoords[j] - box0.m_Verts[0].m_dCoords[j];
//    }
//    else if(box1.m_Verts[0].m_dCoords[j] > box0.m_Verts[1].m_dCoords[j])
//    {
//      vD.m_dCoords[j]= box1.m_Verts[0].m_dCoords[j] - box0.m_Verts[1].m_dCoords[j];
//    }
//    else
//    {
//      vD.m_dCoords[j]=0;
//    }
//
//    sqrDist+=vD.m_dCoords[j]*vD.m_dCoords[j];
//  }//end for
//
//  //if(sqrDist > 0.000025)
//  //{
//  //  std::cout<<"Time early out distance: "<<timer.GetTime()<<std::endl;
//  //  return;
//  //}
//
//  CDistanceMeshMesh<Real> distMeshMesh(&pObject0->m_BVH, &pObject1->m_BVH);
//
//  distMeshMesh.ComputeDistanceEps(0.005);
//
//  //start collision detection
//  std::vector<CTriangle3<Real> > &triangleMesh = pObject0->m_BVH.GetChild(0)->m_Traits.m_vTriangles;
//  Real dist;
//  //timer.Start();
//  for(int i=0;i<triangleMesh.size();i++)
//  {
//    CTriangle3r &tri3_0 = triangleMesh[i];
//    C3DMesh &mesh = pObject1->m_Model.m_vMeshes[0];
//    for(int j=0;j<mesh.GetNumFaces();j++)
//    {
//      CTriangle3r tri3_1(mesh.m_pVertices[mesh.m_pFaces[j][0]],mesh.m_pVertices[mesh.m_pFaces[j][1]],mesh.m_pVertices[mesh.m_pFaces[j][2]]);
//      //compute distance between triangles
//      CDistanceTriangleTriangle<Real> distTriangleTriangle(tri3_0,tri3_1);
//      dist = distTriangleTriangle.ComputeDistance();
//
//      //if the distance is smaller eps
//      if(dist < 0.005)
//      {
//
//        //compute stuff rel. velocity and add
//        VECTOR3 &vPoint = distTriangleTriangle.m_vClosestPoint0;
//
//        //compute the relative velocity
//        VECTOR3 angPart = (VECTOR3::Cross(m_pBody0->GetAngVel(),vPoint-m_pBody0->m_vCOM));
//        VECTOR3 relativeVelocity = (m_pBody0->m_vVelocity + angPart);
//
//        //calculate a good normal
//        VECTOR3 normal = vNormals[j/2];
//
//        //relative velocity along the normal
//        Real normalVelocity = relativeVelocity * normal;
//
//        //check whether there will be a collision next time step
//        if(normalVelocity < 0.0)
//        {
//          //std::cout<<"Pre-contact normal velocity: "<<relVel<<" colliding contact"<<std::endl;
//          CContact contact;
//          contact.m_dDistance  = dist;
//          contact.m_vNormal    = normal;
//          contact.m_vPosition0 = vPoint;
//          contact.m_vPosition1 = vPoint;
//          contact.m_pBody0     = m_pBody0;
//          contact.m_pBody1     = m_pBody1;
//          contact.id0 = contact.m_pBody0->m_iID;
//          contact.id1 = contact.m_pBody1->m_iID;
//          contact.vn           = normalVelocity;
//          vContacts.push_back(contact);
//        }//end if(relVel < 0.0)
//        else if(normalVelocity < 0.00001)
//        {
//          CContact contact;
//          contact.m_dDistance  = dist;
//          contact.m_vNormal    = normal;
//          contact.m_vPosition0 = vPoint;
//          contact.m_vPosition1 = vPoint;
//          contact.m_pBody0     = m_pBody0;
//          contact.m_pBody1     = m_pBody1;
//          contact.id0 = contact.m_pBody0->m_iID;
//          contact.id1 = contact.m_pBody1->m_iID;
//          contact.vn           = normalVelocity;
//          contact.m_pBody0->m_bTouchesGround = true;
//          contact.m_pBody1->m_bTouchesGround = true;
//          vContacts.push_back(contact);
//        }//end else if
//
//      }
//      
//    }
//
//  }//end for
//
////  std::cout<<"Time triangle distance: "<<timer.GetTime()<<std::endl;
//  delete pObject1;
//}

}
