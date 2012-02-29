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

namespace i3d {

CColliderBoxSphere::CColliderBoxSphere() 
{

}

CColliderBoxSphere::~CColliderBoxSphere() 
{

}

void CColliderBoxSphere::Collide(CRigidBody *pBody0, CRigidBody *pBody1, std::vector<CContact> &vContacts, Real dDeltaT)
{
  
    CSpherer *sphere         = dynamic_cast<CSpherer *>(pBody0->m_pShape);
    COBB3r *pBox              = dynamic_cast<COBB3r *>(pBody1->m_pShape);
    
    Real rad0 = sphere->Radius();
    Real rad1 =  pBox->GetBoundingSphereRadius();
    const COBB3r &origBox = dynamic_cast<const COBB3r& >(pBody1->GetOriginalShape());

    VECTOR3 newPos0 = pBody0->m_vCOM + pBody0->m_vVelocity * dDeltaT;
    CPredictionTransform<Real,COBB3r> Transform;
    COBB3r newbox = Transform.PredictMotion(origBox,pBody1->m_vVelocity,pBody1->GetTransformation(),pBody1->GetAngVel(),dDeltaT);

    //in the next time step
    if((newPos0-newbox.m_vCenter).mag() > rad0+rad1)
      return;
    
    MATRIX3X3 old = pBody1->GetTransformation().GetTransformation();
    MATRIX3X3 matAngUpdate = MATRIX3X3::GetSkewMatrix(pBody1->GetAngVel());
		//update orientation
    MATRIX3X3 newOrientation = pBody1->GetTransformation().GetTransformation() + (matAngUpdate *pBody1->GetTransformation().GetTransformation()) * dDeltaT;
    CTransformr newTransform(newOrientation,newbox.m_vCenter);

    CDistancePointObb3<Real> distPointBox(*pBox, newPos0, newTransform);
    Real minDist = distPointBox.ComputeDistance();

    //if the distance in the next timestep is less than the sum of the radii
    if(minDist-rad0 < 0.0)
    {

      //compute the relative velocity in normal direction
      VECTOR3 angPart = (VECTOR3::Cross(pBody1->GetAngVel(),distPointBox.m_vClosestPoint1-pBody1->m_vCOM));
      VECTOR3 relativeVelocity = pBody0->m_vVelocity - pBody1->m_vVelocity - angPart; 

      //relative velocity along the normal
      Real normalVelocity = relativeVelocity * distPointBox.m_ocConf.m_vNormal;

      //between sphere and box there is only one contact point
      //if the bodies are on collision course
      if(normalVelocity < 0.0)
      {
        std::cout<<"Pre-contact normal velocity: "<<normalVelocity<<" colliding contact"<<std::endl;
        CContact contact;
        contact.m_dDistance  = minDist;
        contact.m_vNormal    = distPointBox.m_ocConf.m_vNormal;
        contact.m_vPosition0 = distPointBox.m_vClosestPoint0;
        contact.m_vPosition1 = distPointBox.m_vClosestPoint1;
        contact.m_pBody0     = pBody0;
        contact.m_pBody1     = pBody1;
        contact.vn           = normalVelocity;
        vContacts.push_back(contact);
      }
      else if(normalVelocity < 0.00001)
      {
        std::cout<<"Pre-contact normal velocity: "<<normalVelocity<<" resting contact"<<std::endl;
        CContact contact;
        contact.m_dDistance  = minDist;
        contact.m_vNormal    = distPointBox.m_ocConf.m_vNormal;
        contact.m_vPosition0 = distPointBox.m_vClosestPoint0;
        contact.m_vPosition1 = distPointBox.m_vClosestPoint1;
        contact.m_pBody0     = pBody0;
        contact.m_pBody1     = pBody1;
        contact.vn           = normalVelocity;
        contact.m_bResting   = true;        
        vContacts.push_back(contact);
      }  
    }
  
}

void CColliderBoxSphere::Collide(std::vector<CContact> &vContacts, Real dDeltaT)
{
  
    CSpherer *sphere         = dynamic_cast<CSpherer *>(m_pBody0->m_pShape);
    COBB3r *pBox              = dynamic_cast<COBB3r *>(m_pBody1->m_pShape);
    
    Real rad0 = sphere->Radius();
    Real rad1 =  pBox->GetBoundingSphereRadius();
    const COBB3r &origBox = dynamic_cast<const COBB3r& >(m_pBody1->GetOriginalShape());

    VECTOR3 newPos0 = m_pBody0->m_vCOM;

    //in the next time step
    if((newPos0-m_pBody1->m_vCOM).mag() > rad0+rad1)
      return;
    
    CTransformr newTransform(m_pBody1->GetTransformation().GetTransformation(),m_pBody1->m_vCOM);

    CDistancePointObb3<Real> distPointBox(*pBox, newPos0, newTransform);
    Real minDist = distPointBox.ComputeDistance();
    Real penetrationDepth = minDist-rad0;
    
    //if the distance in the next timestep is less than the sum of the radii
    if(penetrationDepth <= 0.0015)
    {
      //compute the relative velocity in normal direction
      VECTOR3 angPart = (VECTOR3::Cross(m_pBody1->GetAngVel(),distPointBox.m_vClosestPoint1-m_pBody1->m_vCOM));
      VECTOR3 relativeVelocity = m_pBody0->m_vVelocity - m_pBody1->m_vVelocity - angPart; 

      //relative velocity along the normal
      Real normalVelocity = relativeVelocity * distPointBox.m_ocConf.m_vNormal;

      //between sphere and box there is only one contact point
      //if the bodies are on collision course
      if(normalVelocity < -0.005)
      {
        std::cout<<"Pre-contact normal velocity: "<<normalVelocity<<" colliding contact"<<std::endl;
        std::cout<<"Pre-contact angular velocity body0: "<<m_pBody0->GetAngVel()<<std::endl;
        std::cout<<"Pre-contact angular velocity body1: "<<m_pBody1->GetAngVel()<<std::endl;                
        CContact contact;
        contact.m_dDistance  = minDist;
        contact.m_vNormal    = distPointBox.m_ocConf.m_vNormal;
        contact.m_vPosition0 = distPointBox.m_vClosestPoint1;
        contact.m_vPosition1 = distPointBox.m_vClosestPoint1;
        if(m_pBody0->m_iID < m_pBody1->m_iID)
        {
          contact.m_pBody0     = m_pBody0;
          contact.m_pBody1     = m_pBody1;
          contact.id0          = m_pBody0->m_iID;
          contact.id1          = m_pBody1->m_iID;          
          std::cout<<"Contact between: "<<contact.id0<<" "<<contact.id1<<"\n";
          std::cout<<"No switch \n";                              
        }
        else
        {
          contact.m_pBody0     = m_pBody1;
          contact.m_pBody1     = m_pBody0;
          contact.id0          = m_pBody1->m_iID;
          contact.id1          = m_pBody0->m_iID;
          contact.m_vNormal    = -contact.m_vNormal;  
          std::cout<<"Contact between: "<<contact.id0<<" "<<contact.id1<<"\n";          
          std::cout<<"switch \n";                                        
        }          
        contact.vn           = normalVelocity;
        contact.m_iState     = CCollisionInfo::TOUCHING;
        contact.m_dPenetrationDepth = penetrationDepth/dDeltaT;
        vContacts.push_back(contact);
        //std::cout<<"Pre-contact normal velocity: "<<normalVelocity<<" colliding contact"<<std::endl;
        //std::cout<<"Penetration depth: "<<penetrationDepth<<std::endl;
      }
      else if(normalVelocity < 0.00001)
      {
        std::cout<<"Pre-contact normal velocity: "<<normalVelocity<<" resting contact"<<std::endl;
        std::cout<<"Pre-contact angular velocity body0: "<<m_pBody0->GetAngVel()<<std::endl;
        std::cout<<"Pre-contact angular velocity body1: "<<m_pBody1->GetAngVel()<<std::endl;                        
        CContact contact;
        contact.m_dDistance  = minDist;
        contact.m_vNormal    = distPointBox.m_ocConf.m_vNormal;
        contact.m_vPosition0 = distPointBox.m_vClosestPoint1;
        contact.m_vPosition1 = distPointBox.m_vClosestPoint1;
        contact.m_pBody0     = m_pBody0;
        contact.m_pBody1     = m_pBody1;
        contact.id0          = contact.m_pBody0->m_iID;
        contact.id1          = contact.m_pBody1->m_iID;
        contact.vn           = normalVelocity;
        contact.m_iState     = CCollisionInfo::TOUCHING;        
        contact.m_bResting   = true;        
        vContacts.push_back(contact);
      }
      else
      {
        CContact contact;
        contact.m_dDistance  = minDist;
        contact.m_vNormal    = distPointBox.m_ocConf.m_vNormal;
        contact.m_vPosition0 = distPointBox.m_vClosestPoint1;
        contact.m_vPosition1 = distPointBox.m_vClosestPoint1;
        contact.m_pBody0     = m_pBody0;
        contact.m_pBody1     = m_pBody1;
        contact.id0          = contact.m_pBody0->m_iID;
        contact.id1          = contact.m_pBody1->m_iID;
        contact.vn           = normalVelocity;
        contact.m_iState     = CCollisionInfo::VANISHING_CLOSEPROXIMITY;        
        contact.m_bResting   = true;        
        vContacts.push_back(contact);        
      }
    }
}


}