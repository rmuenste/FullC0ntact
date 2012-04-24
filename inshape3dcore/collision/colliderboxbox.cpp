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
	
CColliderBoxBox::CColliderBoxBox() 
{

}

CColliderBoxBox::~CColliderBoxBox() 
{

}

void CColliderBoxBox::Collide(CRigidBody *pBody0, CRigidBody *pBody1, std::vector<CContact> &vContacts, Real dDeltaT)
{


  //here we take the world transformed box
  //TODO: delete these pointers
  COBB3r *pBox0         = dynamic_cast<COBB3r *>(pBody0->GetWorldTransformedShape());

  COBB3r *pBox1         = dynamic_cast<COBB3r *>(pBody1->GetWorldTransformedShape());

  const COBB3r &origBox0 = dynamic_cast<const COBB3r& >(pBody0->GetOriginalShape());
  const COBB3r &origBox1 = dynamic_cast<const COBB3r& >(pBody1->GetOriginalShape());

  VECTOR3 vertices0[8];
  pBox1->ComputeVertices(vertices0);
  bool intersection = false;
  bool intersection2 = false;

  CPredictionTransform<Real,COBB3r> Transform;
  COBB3r newBox0 = Transform.PredictMotion(origBox0,pBody0->m_vVelocity,pBody0->GetTransformation(),pBody0->GetAngVel(),dDeltaT);
  COBB3r newBox1 = Transform.PredictMotion(origBox1,pBody1->m_vVelocity,pBody1->GetTransformation(),pBody1->GetAngVel(),dDeltaT);

  Real rad0 = newBox0.GetBoundingSphereRadius();
  Real rad1 = newBox1.GetBoundingSphereRadius();
  if((newBox0.m_vCenter - newBox1.m_vCenter).mag() > rad0+rad1)
    return;

  VECTOR3 vertices[8];
  newBox1.ComputeVertices(vertices);

  //compute the static intersection query for the next time step
  CIntersector2OBB3r intersectorn1(newBox0,newBox1);
  if(intersectorn1.Test())
  {
    //std::cout<<"detected intersection"<<std::endl;
    intersection = true;
  }

  CIntersector2OBB3r intersector(*pBox0,*pBox1);
  
  std::vector<VECTOR3> &vContactsPoints = intersector.GetContacts();

  VECTOR3 vAxisAngle0 = pBody0->GetAxisAngle();
  VECTOR3 vAxisAngle1 = pBody1->GetAxisAngle();


  //check sat for next time step
  intersection2 = intersector.Find(dDeltaT,3,pBody0->m_vVelocity,pBody1->m_vVelocity,vAxisAngle0,vAxisAngle1);
  intersection2 = false;
  if(intersection && intersection2)
  {
    for(int i=0;i<intersector.GetQuantity();i++)
    {

      CContact contact;
      contact.m_vNormal    = intersector.GetNormal();
      contact.m_vNormal.Normalize();
      contact.m_vPosition0 = vContactsPoints[i];
      contact.m_vPosition1 = vContactsPoints[i];

      if((vContactsPoints[i]-pBody0->m_vCOM)*contact.m_vNormal > 0)
      {
        contact.m_pBody0     = pBody1;
        contact.m_pBody1     = pBody0;
      }
      else
      {
        contact.m_pBody0     = pBody0;
        contact.m_pBody1     = pBody1;
      }

      VECTOR3 vR0 = contact.m_vPosition0-contact.m_pBody0->m_vCOM;
      VECTOR3 vR1 = contact.m_vPosition1-contact.m_pBody1->m_vCOM;

      contact.m_vNormal.Normalize();

      VECTOR3 relativeVelocity = 
        (contact.m_pBody0->m_vVelocity + (VECTOR3::Cross(contact.m_pBody0->GetAngVel(),vR0))
       - contact.m_pBody1->m_vVelocity - (VECTOR3::Cross(contact.m_pBody1->GetAngVel(),vR1)));

      Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);

      //if(intersector.cfg0.m_bPenetration)
      //{
      //  relativeNormalVelocity-=intersector.cfg0.m_dMinOverlap/dDeltaT;
      //}

      if(relativeNormalVelocity <= 0)
      //if(relativeNormalVelocity < -0.005)
      { 
        std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" colliding contact"<<std::endl;
        std::cout<<"Pre-contact angular velocity0: "<<contact.m_pBody0->GetAngVel();
        std::cout<<"Pre-contact angular velocity1: "<<contact.m_pBody1->GetAngVel();
        std::cout<<"Pre-contact  velocity0: "<<contact.m_pBody0->m_vVelocity;
        std::cout<<"Pre-contact  velocity1: "<<contact.m_pBody1->m_vVelocity;
        contact.vn           = relativeNormalVelocity;
        vContacts.push_back(contact);
      }
      else if(relativeNormalVelocity < 0.00001)
      {
//        std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" resting contact"<<std::endl;
        contact.vn           = relativeNormalVelocity;
        vContacts.push_back(contact);
      }
    }//end for
  }
  else if(intersection && !intersection2)
  {
    //CDistanceOBB3OBB3<Real> distObb(*pBox0,*pBox1);
    //std::cout<<distObb.ComputeDistanceSqr()<<std::endl;
    //exit(0);
    //std::cout<<"starting fallback collider"<<std::endl;
    intersectorn1.Find(pBody0->m_vVelocity,pBody1->m_vVelocity,pBody0->GetAngVel(),pBody1->GetAngVel(),dDeltaT);
    for(int i=0;i<intersectorn1.GetQuantity();i++)
    {

      CContact contact;
      contact.m_vNormal    = intersectorn1.GetNormal();
      contact.m_vNormal.Normalize();
      vContactsPoints = intersectorn1.GetContacts();
      contact.m_vPosition0 = vContactsPoints[i];
      contact.m_vPosition1 = vContactsPoints[i];

      if((vContactsPoints[i]-pBody0->m_vCOM)*contact.m_vNormal > 0)
      {
        contact.m_pBody0     = pBody1;
        contact.m_pBody1     = pBody0;
      }
      else
      {
        contact.m_pBody0     = pBody0;
        contact.m_pBody1     = pBody1;
      }

      contact.m_vNormal.Normalize();

      VECTOR3 vR0 = contact.m_vPosition0-contact.m_pBody0->m_vCOM;
      VECTOR3 vR1 = contact.m_vPosition1-contact.m_pBody1->m_vCOM;

      VECTOR3 relativeVelocity = 
        (contact.m_pBody0->m_vVelocity + (VECTOR3::Cross(contact.m_pBody0->GetAngVel(),vR0))
       - contact.m_pBody1->m_vVelocity - (VECTOR3::Cross(contact.m_pBody1->GetAngVel(),vR1)));

      Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);

      //std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" colliding contact"<<std::endl;
      //std::cout<<"Pre-contact angular velocity0: "<<contact.m_pBody0->GetWorldAngVel();
      //std::cout<<"Pre-contact angular velocity1: "<<contact.m_pBody1->GetWorldAngVel();
      //std::cout<<"Pre-contact  velocity0: "<<contact.m_pBody0->m_vVelocity;
      //std::cout<<"Pre-contact  velocity1: "<<contact.m_pBody1->m_vVelocity;

      if(relativeNormalVelocity < -0.005)
      { 
        //std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" colliding contact"<<std::endl;
        contact.vn           = relativeNormalVelocity;
        vContacts.push_back(contact);
      }
      else if(relativeNormalVelocity < 0.00001)
      {
        //std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" resting contact"<<std::endl;
        contact.vn           = relativeNormalVelocity;
        vContacts.push_back(contact);
      }
    }//end for
  }
  
  delete pBox0;
  delete pBox1;

}

//void CColliderBoxBox::Collide(std::vector<CContact> &vContacts, Real dDeltaT)
//{
//
//  if(!m_pBody0->m_bAffectedByGravity && !m_pBody1->m_bAffectedByGravity)
//    return;
//
//  //get reference to the original box
//  const COBB3r &origBox0 = dynamic_cast<const COBB3r& >(m_pBody0->GetOriginalShape());
//  const COBB3r &origBox1 = dynamic_cast<const COBB3r& >(m_pBody1->GetOriginalShape());
// 
//  
//  CPredictionTransform<Real,COBB3r> Transform;
//  COBB3r newBox0 = Transform.PredictMotion(origBox0,m_pBody0->m_vVelocity,m_pBody0->GetTransformation(),m_pBody0->GetAngVel(),dDeltaT);
//  COBB3r newBox1 = Transform.PredictMotion(origBox1,m_pBody1->m_vVelocity,m_pBody1->GetTransformation(),m_pBody1->GetAngVel(),dDeltaT);
//  
//  //check for a quick rejection
//  if((newBox0.m_vCenter - newBox1.m_vCenter).mag() > newBox0.GetBoundingSphereRadius() +newBox1.GetBoundingSphereRadius())
//  {
//    return;
//  }  
//  
//  //here we take the world transformed box
//  COBB3r *pBox0         = dynamic_cast<COBB3r *>(m_pBody0->GetWorldTransformedShape());
//  COBB3r *pBox1         = dynamic_cast<COBB3r *>(m_pBody1->GetWorldTransformedShape());
//
//  bool intersection = false;
//  bool intersection2 = false;
//
//  //more sophisticated intersection test for the next time step
//  CIntersector2OBB3r intersectorNextT(newBox0,newBox1);
//  if(intersectorNextT.Test())
//  {
//    //std::cout<<"detected intersection"<<std::endl;
//    intersection = true;
//  }
//
//  CIntersector2OBB3r intersector(*pBox0,*pBox1);
//  
//  std::vector<VECTOR3> &vContactsPoints = intersector.GetContacts();
//
//  VECTOR3 vAxisAngle0 = m_pBody0->GetAxisAngle();
//  VECTOR3 vAxisAngle1 = m_pBody1->GetAxisAngle();
//
//
//  //check sat for next time step
////   intersection2 = intersector.Find(dDeltaT,3,m_pBody0->m_vVelocity,m_pBody1->m_vVelocity,vAxisAngle0,vAxisAngle1);
////   intersection2 = false;
////   if(intersection && intersection2)
////   {
////     for(int i=0;i<intersector.GetQuantity();i++)
////     {
//// 
////       CContact contact;
////       contact.m_vNormal    = intersector.GetNormal();
////       contact.m_vNormal.Normalize();
////       contact.m_vPosition0 = vContactsPoints[i];
////       contact.m_vPosition1 = vContactsPoints[i];
//// 
////       if((vContactsPoints[i]-m_pBody0->m_vCOM)*contact.m_vNormal > 0)
////       {
////         contact.m_pBody0     = m_pBody1;
////         contact.m_pBody1     = m_pBody0;
////       }
////       else
////       {
////         contact.m_pBody0     = m_pBody0;
////         contact.m_pBody1     = m_pBody1;
////       }
//// 
////       VECTOR3 vR0 = contact.m_vPosition0-contact.m_pBody0->m_vCOM;
////       VECTOR3 vR1 = contact.m_vPosition1-contact.m_pBody1->m_vCOM;
//// 
////       contact.m_vNormal.Normalize();
//// 
////       VECTOR3 relativeVelocity = 
////         (contact.m_pBody0->m_vVelocity + (VECTOR3::Cross(contact.m_pBody0->GetAngVel(),vR0))
////        - contact.m_pBody1->m_vVelocity - (VECTOR3::Cross(contact.m_pBody1->GetAngVel(),vR1)));
//// 
////       Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);
//// 
////       //if(intersector.cfg0.m_bPenetration)
////       //{
////       //  relativeNormalVelocity-=intersector.cfg0.m_dMinOverlap/dDeltaT;
////       //}
//// 
////       if(relativeNormalVelocity <= 0)
////       //if(relativeNormalVelocity < -0.005)
////       { 
////         std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" colliding contact"<<std::endl;
////         std::cout<<"Pre-contact angular velocity0: "<<contact.m_pBody0->GetAngVel();
////         std::cout<<"Pre-contact angular velocity1: "<<contact.m_pBody1->GetAngVel();
////         std::cout<<"Pre-contact  velocity0: "<<contact.m_pBody0->m_vVelocity;
////         std::cout<<"Pre-contact  velocity1: "<<contact.m_pBody1->m_vVelocity;
////         contact.vn           = relativeNormalVelocity;
////         vContacts.push_back(contact);
////       }
////       else if(relativeNormalVelocity < 0.00001)
////       {
//// //        std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" resting contact"<<std::endl;
////         contact.vn           = relativeNormalVelocity;
////         vContacts.push_back(contact);
////       }
////     }//end for
////   }
//
//  //else if(intersection && !intersection2)
//  if(intersection)    
//  {
//    
//    intersectorNextT.Find(m_pBody0->m_vVelocity,m_pBody1->m_vVelocity,m_pBody0->GetAngVel(),m_pBody1->GetAngVel(),dDeltaT);
//
//    //assign the contact points
//    vContactsPoints = intersectorNextT.GetContacts();    
//    
//    //
//    if(intersectorNextT.GetQuantity()==0)
//    {
//      std::cout<<"Error in ColliderBoxBox: Collision detected, but the number of contact points is 0"<<std::endl;
//      exit(0);
//    }
//
//    //loop over the contacts points and generate contact information
//    for(int i=0;i<intersectorNextT.GetQuantity();i++)
//    {
//
//      CContact contact;
//     
//      //assign the contact information
//      contact.m_vNormal    = intersectorNextT.GetNormal();
//      contact.m_pBody0     = m_pBody0;
//      contact.m_pBody1     = m_pBody1;
//      contact.m_vPosition0 = vContactsPoints[i];
//      contact.m_vPosition1 = vContactsPoints[i];
//      contact.id0          = contact.m_pBody0->m_iID;
//      contact.id1          = contact.m_pBody1->m_iID;
//      
//      //the computed normal might not be normalized
//      contact.m_vNormal.Normalize();
//
//      //and it might not point in the direction
//      //we choose by convention
//      if((vContactsPoints[i]-m_pBody0->m_vCOM)*contact.m_vNormal > 0)
//      {
//        contact.m_vNormal = -contact.m_vNormal;
//      }
//
//      //compute the normal velocity and classify the contact point
//      VECTOR3 vR0 = contact.m_vPosition0-contact.m_pBody0->m_vCOM;
//      VECTOR3 vR1 = contact.m_vPosition1-contact.m_pBody1->m_vCOM;
//
//      VECTOR3 relativeVelocity = 
//        (contact.m_pBody0->m_vVelocity + (VECTOR3::Cross(contact.m_pBody0->GetAngVel(),vR0))
//       - contact.m_pBody1->m_vVelocity - (VECTOR3::Cross(contact.m_pBody1->GetAngVel(),vR1)));
//
//      Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);
//
//      //std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<std::endl;
//      //std::cout<<"Pre-contact angular velocity0: "<<contact.m_pBody0->GetWorldAngVel();
//      //std::cout<<"Pre-contact angular velocity1: "<<contact.m_pBody1->GetWorldAngVel();
//      //std::cout<<"Pre-contact  velocity0: "<<contact.m_pBody0->m_vVelocity;
//      //std::cout<<"Pre-contact  velocity1: "<<contact.m_pBody1->m_vVelocity;
//
//      //we declare a contact as resting if the velocity is below a
//      //small negative tolerance. The velocity for resting contacts can be negative, because
//      //of inaccuracies in the contact point generation
//      if(relativeNormalVelocity < -0.005)
//      { 
//        //std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" colliding contact"<<std::endl;
//        contact.vn           = relativeNormalVelocity;
//        contact.m_iState     = CCollisionInfo::TOUCHING;
//        vContacts.push_back(contact);
//      }
//      else if(relativeNormalVelocity < 0.00001)
//      {
//        //std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" resting contact"<<std::endl;
//        contact.vn           = relativeNormalVelocity;
//        contact.m_iState     = CCollisionInfo::TOUCHING;
//        vContacts.push_back(contact);
//      }
//      else
//      {
//        //the relative velocity is greater than eps
//        contact.vn           = relativeNormalVelocity;
//        contact.m_iState     = CCollisionInfo::VANISHING_CLOSEPROXIMITY;        
//        vContacts.push_back(contact);        
//      }
//    }//end for
//  }
//  
//  delete pBox0;
//  delete pBox1;
//
//}

void CColliderBoxBox::Collide(std::vector<CContact> &vContacts, Real dDeltaT)
{

  if(!m_pBody0->m_bAffectedByGravity && !m_pBody1->m_bAffectedByGravity)
    return;

  //get reference to the original box
  const COBB3r &origBox0 = dynamic_cast<const COBB3r& >(m_pBody0->GetOriginalShape());
  const COBB3r &origBox1 = dynamic_cast<const COBB3r& >(m_pBody1->GetOriginalShape());
  
  //here we take the world transformed box
  COBB3r *pBox0         = dynamic_cast<COBB3r *>(m_pBody0->GetWorldTransformedShape());
  COBB3r *pBox1         = dynamic_cast<COBB3r *>(m_pBody1->GetWorldTransformedShape());

  //check for a quick rejection
  if((pBox0->m_vCenter - pBox1->m_vCenter).mag() > pBox0->GetBoundingSphereRadius() + pBox1->GetBoundingSphereRadius())
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
  if(intersectorNextT.Test2(m_pBody0->m_vVelocity,m_pBody1->m_vVelocity))
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

      CContact contact;
     
      //assign the contact information
      contact.m_vNormal    = intersectorNextT.GetNormal();
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.m_vPosition0 = vContactsPoints[i];
      contact.m_vPosition1 = vContactsPoints[i];
      contact.id0          = contact.m_pBody0->m_iID;
      contact.id1          = contact.m_pBody1->m_iID;
      
      //the computed normal might not be normalized
      contact.m_vNormal.Normalize();

      //and it might not point in the direction
      //we choose by convention
      if((vContactsPoints[i]-m_pBody0->m_vCOM)*contact.m_vNormal > 0)
      {
        contact.m_vNormal = -contact.m_vNormal;
      }

      //compute the normal velocity and classify the contact point
      VECTOR3 vR0 = contact.m_vPosition0-contact.m_pBody0->m_vCOM;
      VECTOR3 vR1 = contact.m_vPosition1-contact.m_pBody1->m_vCOM;

      VECTOR3 relativeVelocity = 
        (contact.m_pBody0->m_vVelocity + (VECTOR3::Cross(contact.m_pBody0->GetAngVel(),vR0))
       - contact.m_pBody1->m_vVelocity - (VECTOR3::Cross(contact.m_pBody1->GetAngVel(),vR1)));

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
        contact.m_iState     = CCollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }
      else if(relativeNormalVelocity < 0.00001)
      {
        //std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" resting contact"<<std::endl;
        contact.vn           = relativeNormalVelocity;
        contact.m_iState     = CCollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }
      else if(relativeNormalVelocity > 1001)
      {
        //std::cout<<"Pre-contact normal velocity: "<<relativeNormalVelocity<<" resting contact"<<std::endl;
        contact.vn           = relativeNormalVelocity;
        contact.m_iState     = CCollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }
      else
      {
        //the relative velocity is greater than eps
        contact.vn           = relativeNormalVelocity;
        contact.m_iState     = CCollisionInfo::VANISHING_CLOSEPROXIMITY;        
        vContacts.push_back(contact);        
      }
    }//end for
  }
  
  delete pBox0;
  delete pBox1;

}


}
