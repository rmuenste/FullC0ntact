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
#include "colliderboxsphere.h"
#include <obb3.h>
#include <sphere.h>
#include <distancepointobb3.h>
#include <contact.h>
#include "collisioninfo.h"
#include <world.h>

namespace i3d {

ColliderBoxSphere::ColliderBoxSphere() 
{

}

ColliderBoxSphere::~ColliderBoxSphere() 
{

}

void ColliderBoxSphere::collide(std::vector<Contact> &vContacts)
{
  
    Spherer *sphere         = dynamic_cast<Spherer *>(body0_->shape_);
    OBB3r *pBox              = dynamic_cast<OBB3r *>(body1_->shape_);
    
    Real rad0 = sphere->getRadius();
    Real rad1 =  pBox->getBoundingSphereRadius();
    const OBB3r &origBox = dynamic_cast<const OBB3r& >(body1_->getOriginalShape());

    VECTOR3 newPos0 = body0_->com_;

    //in the next time step
    if((newPos0-body1_->com_).mag() > rad0+rad1)
      return;
    
    Transformationr newTransform(body1_->getTransformation().getMatrix(),body1_->com_);

    CDistancePointObb3<Real> distPointBox(*pBox, newPos0, newTransform);
    Real minDist = distPointBox.ComputeDistance();
    Real penetrationDepth = minDist-rad0;
    
    //if the distance in the next timestep is less than the sum of the radii
    if(penetrationDepth <= 0.0015)
    {
      //compute the relative velocity in normal direction
      VECTOR3 angPart = (VECTOR3::Cross(body1_->getAngVel(),distPointBox.m_vClosestPoint1-body1_->com_));
      VECTOR3 relativeVelocity = body0_->velocity_ - body1_->velocity_ - angPart; 

      //relative velocity along the normal
      Real normalVelocity = relativeVelocity * distPointBox.m_ocConf.m_vNormal;

      //between sphere and box there is only one contact point
      //if the bodies are on collision course
      if(normalVelocity < -0.005)
      {
        Contact contact;
        contact.m_dDistance  = minDist;
        contact.m_vNormal    = distPointBox.m_ocConf.m_vNormal;
        contact.m_vPosition0 = distPointBox.m_vClosestPoint1;
        contact.m_vPosition1 = distPointBox.m_vClosestPoint1;
        if(body0_->iID_ < body1_->iID_)
        {
          contact.m_pBody0     = body0_;
          contact.m_pBody1     = body1_;
          contact.id0          = body0_->iID_;
          contact.id1          = body1_->iID_;          
         // std::cout<<"Contact between: "<<contact.id0<<" "<<contact.id1<<"\n";
         // std::cout<<"No switch \n";                              
        }
        else
        {
          contact.m_pBody0     = body1_;
          contact.m_pBody1     = body0_;
          contact.id0          = body1_->iID_;
          contact.id1          = body0_->iID_;
          contact.m_vNormal    = -contact.m_vNormal;  
          //std::cout<<"Contact between: "<<contact.id0<<" "<<contact.id1<<"\n";          
          //std::cout<<"switch \n";                                        
        }          
        contact.vn           = normalVelocity;
        contact.m_iState     = CollisionInfo::TOUCHING;
        contact.m_dPenetrationDepth = penetrationDepth/world_->timeControl_->GetDeltaT();
        vContacts.push_back(contact);
        //std::cout<<"Pre-contact normal velocity: "<<normalVelocity<<" colliding contact"<<std::endl;
        //std::cout<<"Penetration depth: "<<penetrationDepth<<std::endl;
      }
      else if(normalVelocity < 0.00001)
      {
        Contact contact;
        contact.m_dDistance  = minDist;
        contact.m_vNormal    = distPointBox.m_ocConf.m_vNormal;
        contact.m_vPosition0 = distPointBox.m_vClosestPoint1;
        contact.m_vPosition1 = distPointBox.m_vClosestPoint1;
        contact.m_pBody0     = body0_;
        contact.m_pBody1     = body1_;
        contact.id0          = contact.m_pBody0->iID_;
        contact.id1          = contact.m_pBody1->iID_;
        contact.vn           = normalVelocity;
        contact.m_iState     = CollisionInfo::TOUCHING;        
        vContacts.push_back(contact);
      }
    }
    else if(penetrationDepth < 0.2*rad0)
    {
      VECTOR3 angPart = (VECTOR3::Cross(body1_->getAngVel(),distPointBox.m_vClosestPoint1-body1_->com_));
      VECTOR3 relativeVelocity = body0_->velocity_ - body1_->velocity_ - angPart; 

      //relative velocity along the normal
      Real normalVelocity = relativeVelocity * distPointBox.m_ocConf.m_vNormal;

      Contact contact;
      contact.m_dDistance  = minDist;
      contact.m_vNormal    = distPointBox.m_ocConf.m_vNormal;
      contact.m_vPosition0 = distPointBox.m_vClosestPoint1;
      contact.m_vPosition1 = distPointBox.m_vClosestPoint1;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0          = contact.m_pBody0->iID_;
      contact.id1          = contact.m_pBody1->iID_;
      contact.vn           = normalVelocity;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }
    else
    {
      return;
      Contact contact;
      contact.m_dDistance  = minDist;
      contact.m_vNormal    = distPointBox.m_ocConf.m_vNormal;
      contact.m_vPosition0 = distPointBox.m_vClosestPoint1;
      contact.m_vPosition1 = distPointBox.m_vClosestPoint1;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0          = contact.m_pBody0->iID_;
      contact.id1          = contact.m_pBody1->iID_;
      contact.m_iState     = CollisionInfo::VANISHING_CLOSEPROXIMITY;
      vContacts.push_back(contact);
    }
}


}
