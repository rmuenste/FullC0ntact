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

ColliderMeshBoundaryBox::ColliderMeshBoundaryBox() 
{

}

ColliderMeshBoundaryBox::~ColliderMeshBoundaryBox() 
{

}

void ColliderMeshBoundaryBox::collide(std::vector<Contact> &vContacts)
{

  CMeshObjectr *pMeshObjectOrig = dynamic_cast<CMeshObjectr*>(body0_->shape_);
  BoundaryBoxr *pBoundary  = dynamic_cast<BoundaryBoxr *>(body1_->shape_);

  if(!body0_->isAffectedByGravity())
    return;

  //now check for all walls
  for(int k=0;k<6;k++)
  {
    //calculate the distance
    int indexOrigin = k/2;

    Real dist=0;

    double timeCp=0.0;

    Planer plane(pBoundary->points_[k],pBoundary->normals_[k]);
    CDistanceModelPlane<Real> distModelPlane(&plane,&pMeshObjectOrig->m_BVH);
    distModelPlane.ComputeDistanceEps(0.01);

    std::vector<VECTOR3>::iterator viter = distModelPlane.m_vPoint.begin();
    for(;viter!=distModelPlane.m_vPoint.end();viter++)
    {
      VECTOR3 &vPoint = *viter;

      //compute the relative velocity
      VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),vPoint-body0_->com_));
      VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

      //relative velocity along the normal
      Real normalVelocity = relativeVelocity * pBoundary->normals_[k];

      //check whether there will be a collision next time step
      if(normalVelocity < 0.0)
      {
        //std::cout<<"Pre-contact normal velocity: "<<relVel<<" colliding contact"<<std::endl;
        Contact contact;
        contact.m_dDistance  = dist;
        contact.m_vNormal    = pBoundary->normals_[k];
        contact.m_vPosition0 = vPoint;
        contact.m_vPosition1 = vPoint;
        contact.m_pBody0     = body0_;
        contact.m_pBody1     = body1_;
        contact.id0          = contact.m_pBody0->iID_;
        contact.id1          = contact.m_pBody1->iID_;
        contact.vn           = normalVelocity;
        contact.m_iState     = CollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }//end if(relVel < 0.0)
      else if(normalVelocity < 0.00001)
      {
        Contact contact;
        contact.m_dDistance  = dist;
        contact.m_vNormal    = pBoundary->normals_[k];
        contact.m_vPosition0 = vPoint;
        contact.m_vPosition1 = vPoint;
        contact.m_pBody0     = body0_;
        contact.m_pBody1     = body1_;
        contact.id0          = contact.m_pBody0->iID_;
        contact.id1          = contact.m_pBody1->iID_;
        contact.vn           = normalVelocity;
        contact.m_iState     = CollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }//end else if      
    }//end viter

  }//end for all walls

}

}
