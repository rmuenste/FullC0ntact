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


#include "colliderboxbox.h"
#include <distanceobb3obb3.h>
#include <intersector2obb3.h>
#include <transform.h>
#include "collisioninfo.h"

namespace i3d {
	
ColliderBoxBox::ColliderBoxBox() 
{

}

ColliderBoxBox::~ColliderBoxBox() 
{

}

void ColliderBoxBox::collide(std::vector<Contact> &vContacts)
{

  if(!body0_->affectedByGravity_ && !body1_->affectedByGravity_)
    return;

  //get reference to the original box
  const OBB3r &origBox0 = dynamic_cast<const OBB3r& >(body0_->getOriginalShape());
  const OBB3r &origBox1 = dynamic_cast<const OBB3r& >(body1_->getOriginalShape());
  
  //here we take the world transformed box
  OBB3r *pBox0         = dynamic_cast<OBB3r *>(body0_->getWorldTransformedShape());
  OBB3r *pBox1         = dynamic_cast<OBB3r *>(body1_->getWorldTransformedShape());

  //check for a quick rejection
  if((pBox0->center_ - pBox1->center_).mag() > pBox0->getBoundingSphereRadius() + pBox1->getBoundingSphereRadius())
  {
    delete pBox0;
    delete pBox1;    
    return;
  }  

  bool intersection = false;
  bool intersection2 = false;

  //more sophisticated intersection test for the next time step
  CIntersector2OBB3r intersectorNextT(*pBox0,*pBox1);
  std::vector<VECTOR3> &vContactsPoints = intersectorNextT.GetContacts();
  if(intersectorNextT.Test2(body0_->velocity_,body1_->velocity_))
  {
    //std::cout<<"detected intersection"<<std::endl;
    intersection = true;
  }

  if(intersection)    
  {
    
    //assign the contact points
    vContactsPoints = intersectorNextT.GetContacts();    
    if(intersectorNextT.GetQuantity()==0)
    {
      std::cout<<"Error in ColliderBoxBox: Collision detected, but the number of contact points is 0"<<std::endl;
      //exit(0);
    }

    //loop over the contacts points and generate contact information
    for(int i=0;i<intersectorNextT.GetQuantity();i++)
    {

      Contact contact;
     
      //assign the contact information
      contact.m_vNormal    = intersectorNextT.GetNormal();
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.m_vPosition0 = vContactsPoints[i];
      contact.m_vPosition1 = vContactsPoints[i];
      contact.id0          = contact.m_pBody0->iID_;
      contact.id1          = contact.m_pBody1->iID_;
      
      //the computed normal might not be normalized
      contact.m_vNormal.Normalize();

      //and it might not point in the direction
      //we choose by convention
      if((vContactsPoints[i]-body0_->com_)*contact.m_vNormal > 0)
      {
        contact.m_vNormal = -contact.m_vNormal;
      }

      //compute the normal velocity and classify the contact point
      VECTOR3 vR0 = contact.m_vPosition0-contact.m_pBody0->com_;
      VECTOR3 vR1 = contact.m_vPosition1-contact.m_pBody1->com_;

      VECTOR3 relativeVelocity = 
        (contact.m_pBody0->velocity_ + (VECTOR3::Cross(contact.m_pBody0->getAngVel(),vR0))
       - contact.m_pBody1->velocity_ - (VECTOR3::Cross(contact.m_pBody1->getAngVel(),vR1)));

      Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);

      //std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<std::endl;
      //std::cout<<"Pre-contact angular velocity0: "<<contact.m_pBody0->GetWorldAngVel();
      //std::cout<<"Pre-contact angular velocity1: "<<contact.m_pBody1->GetWorldAngVel();
      //std::cout<<"Pre-contact  velocity0: "<<contact.m_pBody0->m_vVelocity;
      //std::cout<<"Pre-contact  velocity1: "<<contact.m_pBody1->m_vVelocity;

      //we declare a contact as resting if the velocity is below a
      //small negative tolerance. The velocity for resting contacts can be negative, because
      //of inaccuracies in the contact point generation
      if(relativeNormalVelocity < -0.005)
      { 
        //std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" colliding contact"<<std::endl;
        contact.vn           = relativeNormalVelocity;
        contact.m_iState     = CollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }
      else if(relativeNormalVelocity < 0.00001)
      {
        //std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" resting contact"<<std::endl;
        contact.vn           = relativeNormalVelocity;
        contact.m_iState     = CollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }
      else if(relativeNormalVelocity > 1001)
      {
        //std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" resting contact"<<std::endl;
        contact.vn           = relativeNormalVelocity;
        contact.m_iState     = CollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }
      else
      {
        //the relative velocity is greater than eps
        contact.vn           = relativeNormalVelocity;
        contact.m_iState     = CollisionInfo::VANISHING_CLOSEPROXIMITY;        
        vContacts.push_back(contact);        
      }
    }//end for
  }
  
  delete pBox0;
  delete pBox1;

}

}
