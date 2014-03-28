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
#include <distanceaabbaabb.h>
#include <distancemap.h>
#include "collisioninfo.h"
#include <vtkwriter.h>
#include <iomanip>
#include <sstream>
#include <world.h>

namespace i3d {

ColliderMeshMesh::ColliderMeshMesh() 
{

}

ColliderMeshMesh::~ColliderMeshMesh() 
{

}

/**
* @see CCollider::Collide
*
*/  
void ColliderMeshMesh::collide(std::vector<Contact> &vContacts)
{

  CMeshObject<Real> *pObject0 = dynamic_cast<CMeshObject<Real>* >(body0_->shape_);
  CMeshObject<Real> *pObject1 = dynamic_cast<CMeshObject<Real>* >(body1_->shape_);
  
  DistanceMap<Real> *map0 = body0_->map_;
  
  CBoundingVolumeTree3<AABB3<Real>,Real,CTraits,CSubdivisionCreator> *pBVH = &pObject1->m_BVH;  
  CBoundingVolumeTree3<AABB3<Real>,Real,CTraits,CSubdivisionCreator> *pBVH0 = &pObject0->m_BVH;  

  //std::cout<<"ColliderMeshMesh: Checking distance map"<<std::endl;
   
  //get all the triangles contained in the root node
  Real mindist = CMath<Real>::MAXREAL;
  VECTOR3 cp0(0,0,0);
  VECTOR3 cp_pre(0,0,0);
  VECTOR3 cp_dm(0,0,0);  
  int tri=-1;
  
  CBoundingVolumeNode3<AABB3<Real>,Real,CTraits> *pNode = pBVH->GetChild(0);
  CBoundingVolumeNode3<AABB3<Real>,Real,CTraits> *pNode0 = pBVH0->GetChild(0);
  int s = pNode->m_Traits.m_vTriangles.size();

  AABB3r *box = &pNode->m_BV;
  CDistanceAabbAabb<Real> distance(pNode->m_BV,pNode0->m_BV);
  
  Real boxDistance = distance.ComputeDistanceSqr();
  
  if(boxDistance > 0.02 * 0.02)
    return;
  
  std::vector<Triangle3r> vTriangles0 = pNode0->m_Traits.m_vTriangles;
  std::vector<Triangle3r> vTriangles1 = pNode->m_Traits.m_vTriangles;

  Transformationr World2Model = body0_->getTransformation();
  MATRIX3X3 Model2World = World2Model.getMatrix();
  World2Model.Transpose();

  for(int i=0;i<vTriangles1.size();i++)
  {
    vTriangles1[i].m_vV0 = World2Model.getMatrix() * (vTriangles1[i].m_vV0 - World2Model.getOrigin());
    vTriangles1[i].m_vV1 = World2Model.getMatrix() * (vTriangles1[i].m_vV1 - World2Model.getOrigin());
    vTriangles1[i].m_vV2 = World2Model.getMatrix() * (vTriangles1[i].m_vV2 - World2Model.getOrigin());
  }

  for(int i=0;i<vTriangles0.size();i++)
  {
    vTriangles0[i].m_vV0 = World2Model.getMatrix() * (vTriangles0[i].m_vV0 - World2Model.getOrigin());
    vTriangles0[i].m_vV1 = World2Model.getMatrix() * (vTriangles0[i].m_vV1 - World2Model.getOrigin());
    vTriangles0[i].m_vV2 = World2Model.getMatrix() * (vTriangles0[i].m_vV2 - World2Model.getOrigin());
  }

//   std::ostringstream sNumber;
//   std::string sTrianglesA("output/trianglesA.vtk");
//   std::string sTrianglesB("output/trianglesB.vtk");
//   int iTimestep=this->m_pWorld->m_pTimeControl->m_iTimeStep;
//   sNumber<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
//   sTrianglesA.append(sNumber.str());
//   sTrianglesB.append(sNumber.str());    
//   
//   CVtkWriter writer;
//   writer.WriteTriangles(vTriangles0,sTrianglesA.c_str());
//   writer.WriteTriangles(vTriangles1,sTrianglesB.c_str());
      
  for(int k=0;k<pNode->m_Traits.m_vTriangles.size();k++)
  {
    Triangle3<Real> &tri3 = pNode->m_Traits.m_vTriangles[k];
    VECTOR3 points[3];
    points[0] = tri3.m_vV0;
    points[1] = tri3.m_vV1;        
    points[2] = tri3.m_vV2;              
                  
    for(int l=0;l<3;l++)
    {
      //transform the points into distance map coordinate system        
      VECTOR3 vQuery=points[l];
      vQuery = World2Model.getMatrix() * (vQuery - World2Model.getOrigin());        
      std::pair<Real, CVector3<Real> > result;
      result = map0->queryMap(vQuery);
      //if distance < eps
      //add contact point
      if(mindist > result.first)
      {
        mindist=result.first;
        cp0=vQuery;
        //cp0=(Model2World*vQuery)+World2Model.getOrigin();
        cp_pre=points[l];
        cp_dm=result.second;
        tri=k;
      }
          
      //add contact point
      //check whether there will be a collision next time step
      if(result.first < 0.02)
      {
        VECTOR3 c0 = (Model2World * cp_dm) + World2Model.getOrigin();
        VECTOR3 c1 = (Model2World * cp0) + World2Model.getOrigin();
        
        //std::cout<<"Pre-contact normal velocity: "<<relVel<<" colliding contact"<<std::endl;
        Contact contact;
        contact.m_dDistance  = mindist;
        contact.m_vPosition0 = 0.5 * (c0 + c1);
        contact.m_vPosition1 = contact.m_vPosition0;
            
        contact.m_vNormal    = c0 - c1;
        contact.m_vNormal.Normalize();
        
        contact.m_pBody0     = body0_;
        contact.m_pBody1     = body1_;
        contact.id0          = contact.m_pBody0->iID_;
        contact.id1          = contact.m_pBody1->iID_;

        contact.m_iState     = CollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }//end if(relVel < 0.0)                   
    }                
  }//end for k

  std::vector<VECTOR3> closest_pair;  
  closest_pair.push_back((Model2World * cp_dm) + World2Model.getOrigin());
  closest_pair.push_back((Model2World * cp0) + World2Model.getOrigin());

//   std::ostringstream sName;
//   std::string sModel("output/cpoints.vtk");
//   sName<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
//   sModel.append(sName.str());
// 
//   writer.WritePoints(closest_pair,sModel.c_str());
    
//   std::cout<<"Minimal distance: "<<mindist<<std::endl;
//   std::cout<<"Closest point(transformed): "<<cp0<<std::endl;
//   std::cout<<"Closest point(distance map): "<<cp_dm<<std::endl;  
//   std::cout<<"Original point: "<<cp_pre<<std::endl;
//   std::cout<<"Triangle: "<<tri<<std::endl;
//   std::cout<<"Triangle p1: "<<pNode->m_Traits.m_vTriangles[tri].m_vV0;        
//   std::cout<<"Triangle p2: "<<pNode->m_Traits.m_vTriangles[tri].m_vV1;        
//   std::cout<<"Triangle p3: "<<pNode->m_Traits.m_vTriangles[tri].m_vV2;        
  
}

}
