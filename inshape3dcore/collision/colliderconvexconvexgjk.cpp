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


#include "colliderconvexconvexgjk.h"
#include <distanceconvexconvexgjk.h>
#include <cylinder.h>
#include <intersectortools.h>
#include <intersectormpr.h>
#include "collisioninfo.h"

namespace i3d {

ColliderConvexConvexGjk::ColliderConvexConvexGjk() 
{

}

ColliderConvexConvexGjk::~ColliderConvexConvexGjk() 
{

}
 
void ColliderConvexConvexGjk::collide(std::vector<Contact> &vContacts)
{
  
  //transform to next time step
  //here we take the world transformed box
  //TODO: delete these pointers
  ConvexShaper *pConvex0             = dynamic_cast<ConvexShaper *>(body0_->getWorldTransformedShape());
  ConvexShaper *pConvex1             = dynamic_cast<ConvexShaper *>(body1_->getWorldTransformedShape());

  const ConvexShaper &origConvex0 = dynamic_cast<const ConvexShaper& >(body0_->getOriginalShape());
  const ConvexShaper &origConvex1 = dynamic_cast<const ConvexShaper& >(body1_->getOriginalShape());

  const Real PROXIMITYCOLLISION = 0.007; 
  const Real COLLIDINGTOLERANCE = 0.0;

  //CPredictionTransform<Real,COBB3r> Transform;
  //COBB3r newBox0 = Transform.PredictMotion(origBox0,pBody0->m_vVelocity,pBody0->GetTransformation(),pBody0->GetAngVel(),dDeltaT);
  //COBB3r newBox1 = Transform.PredictMotion(origBox1,pBody1->m_vVelocity,pBody1->GetTransformation(),pBody1->GetAngVel(),dDeltaT);

  CDistanceConvexConvexGjk<Real> gjk(*pConvex0,*pConvex1,body0_->getTransformation(),
                                                         body1_->getTransformation());

  Real dist = gjk.ComputeDistance();

  //start the minkowski portal algorithm
  CIntersectorMPR<Real> intersector(*pConvex0,*pConvex1);

  if(dist <= PROXIMITYCOLLISION)
  {
    //in case the objects are close, this way to caculate the normal can produce unwanted results due to numerical inaccuracies.
    //but it is still good enough to serve as a temporary approximate of the normal
    //during the contact point generation the normal is corrected
    VECTOR3 vNormal = gjk.m_vClosestPoint0 - gjk.m_vClosestPoint1; //VECTOR3(0,0,-1);
    vNormal.Normalize();
    int nContacts=0;
    std::vector<VECTOR3> vContactPoints;

    generator_->GenerateContactPoints(origConvex0,origConvex1,gjk.m_vSimplex,
                                        body0_->getTransformation(),body1_->getTransformation(),
                                        gjk.m_vClosestPoint0,gjk.m_vClosestPoint1,vNormal,nContacts,vContactPoints);

//    vContactPoints.push_back(gjk.m_vClosestPoint1);
    //std::cout<<"Number of contact points: "<<vContactPoints.size()<<std::endl;
    for(int i=0;i<vContactPoints.size();i++)
    {

      //compute relative velocity;
      VECTOR3 v0 = vContactPoints[i] - body0_->com_;
      VECTOR3 v1 = vContactPoints[i] - body1_->com_;

      //compute angular part
      VECTOR3 angPart          = VECTOR3::Cross(body0_->getAngVel(),v0) - VECTOR3::Cross(body1_->getAngVel(),v1);

      //add translational part
      VECTOR3 relativeVelocity = body0_->velocity_ - body1_->velocity_ + angPart;

      Real normalVelocity = relativeVelocity * vNormal;
      //std::cout<<"Pre-contact normal velocity: "<<normalVelocity<<" resting contact"<<std::endl;      

      Contact contact;
      contact.m_vNormal    = vNormal;
      contact.m_vPosition0 = vContactPoints[i];
      contact.m_vPosition1 = vContactPoints[i];
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0          = contact.m_pBody0->iID_;
      contact.id1          = contact.m_pBody1->iID_;
      contact.vn           = normalVelocity;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);

    }
  }

  delete pConvex0;
  delete pConvex1;
  
}

}
