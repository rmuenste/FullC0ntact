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
#include "collidermeshboundarybox.h"
#include <boundarybox.h>
#include <meshobject.h>
#include <distancemodelplane.h>
#include <perftimer.h>
#include "collisioninfo.h"

namespace i3d {

CColliderMeshBoundaryBox::CColliderMeshBoundaryBox() 
{

}

CColliderMeshBoundaryBox::~CColliderMeshBoundaryBox() 
{

}

void CColliderMeshBoundaryBox::Collide(CRigidBody *pBody0, CRigidBody *pBody1, std::vector<CContact> &vContacts, Real dDeltaT)
{
	std::cout<<"Collide"<<std::endl;
}

void CColliderMeshBoundaryBox::Collide(std::vector<CContact> &vContacts, Real dDeltaT)
{

  CMeshObjectr *pMeshObjectOrig = dynamic_cast<CMeshObjectr*>(m_pBody0->m_pShape);
  CBoundaryBoxr *pBoundary  = dynamic_cast<CBoundaryBoxr *>(m_pBody1->m_pShape);

  if(!m_pBody0->IsAffectedByGravity())
    return;

  //now check for all walls
  for(int k=0;k<6;k++)
  {
    //calculate the distance
    int indexOrigin = k/2;

    Real dist=0;

    double timeCp=0.0;
//     CPerfTimer timer0;
//     timer0.Start();

    CPlaner plane(pBoundary->m_vPoints[k],pBoundary->m_vNormals[k]);
    CDistanceModelPlane<Real> distModelPlane(&plane,&pMeshObjectOrig->m_BVH);
    distModelPlane.ComputeDistanceEps(0.005);


    std::vector<VECTOR3>::iterator viter = distModelPlane.m_vPoint.begin();
    for(;viter!=distModelPlane.m_vPoint.end();viter++)
    {
      VECTOR3 &vPoint = *viter;

      //compute the relative velocity
      VECTOR3 angPart = (VECTOR3::Cross(m_pBody0->GetAngVel(),vPoint-m_pBody0->m_vCOM));
      VECTOR3 relativeVelocity = (m_pBody0->m_vVelocity + angPart);

      //relative velocity along the normal
      Real normalVelocity = relativeVelocity * pBoundary->m_vNormals[k];

      //check whether there will be a collision next time step
      if(normalVelocity < 0.0)
      {
        //std::cout<<"Pre-contact normal velocity: "<<relVel<<" colliding contact"<<std::endl;
        CContact contact;
        contact.m_dDistance  = dist;
        contact.m_vNormal    = pBoundary->m_vNormals[k];
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
        contact.m_vNormal    = pBoundary->m_vNormals[k];
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
        contact.m_vNormal    = pBoundary->m_vNormals[k];
        contact.m_vPosition0 = vPoint;
        contact.m_vPosition1 = vPoint;
        contact.m_pBody0     = m_pBody0;
        contact.m_pBody1     = m_pBody1;
        contact.id0          = contact.m_pBody0->m_iID;
        contact.id1          = contact.m_pBody1->m_iID;
        contact.vn           = normalVelocity;
        contact.m_iState     = CCollisionInfo::VANISHED_TOUCHING;
        vContacts.push_back(contact);        
      }
      
    }//end viter

//     timeCp=timer0.GetTime();
//     if(!distModelPlane.m_vPoint.empty())
//       std::cout<<"Time cps: "<<timeCp<<std::endl;


  }//end for all walls

}

}
